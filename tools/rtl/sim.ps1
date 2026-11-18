<#
.SYNOPSIS
    Verilator simulation build and runner on Windows (PowerShell).

.DESCRIPTION
    Compiles RTL into an executable simulation via Verilator + MinGW GCC 14.2.0.
    Handles all Windows-specific gotchas:
      * VERILATOR_ROOT and PATH (bin + lib) setup
      * Dynamic resolution of WinLibs GCC 14.2.0 (and warning against GCC >= 16)
      * make = WinLibs mingw32-make (via MAKE), POSIX sh from MSYS2; AndeSight's
        cygwin is stripped from PATH and never needed
      * Space-free build directory (GNU Make in Verilator fails on paths with spaces)
      * Stderr and stdout capture via Start-Process
      * Clean execution of the compiled binary

.EXAMPLE
    # Self-contained Verilog with built-in main
    .\sim.ps1 -Top top -Sources src\top.v

.EXAMPLE
    # With a C++ testbench and VCD tracing
    .\sim.ps1 -Top top -Sources src\top.v -CppMain sim\sim_main.cpp -Trace
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]   $Top,
    [Parameter(Mandatory)][string[]] $Sources,
    [string]   $CppMain,
    [string]   $OssRoot = "$env:USERPROFILE\oss-cad-suite",
    [string[]] $Waive   = @('DECLFILENAME', 'UNUSEDSIGNAL', 'UNUSEDPARAM',
                            'VARHIDDEN', 'PINCONNECTEMPTY'),
    [switch]   $BlackboxMissing,
    [switch]   $Timing,
    [switch]   $Trace,
    [string[]] $ExtraArgs = @(),
    [string[]] $RunArgs   = @(),
    [string]   $BuildDir,
    [switch]   $NoRun
)

$ErrorActionPreference = 'Stop'

# 1. Locate Verilator
if (-not (Test-Path "$OssRoot\bin\verilator_bin.exe")) {
    throw "verilator_bin.exe not found under $OssRoot. Pass -OssRoot."
}
# Forward slashes: the generated makefile hands $(VERILATOR_ROOT) to sh in the
# `python3 $(VERILATOR_INCLUDER)` recipe, and sh eats every backslash - python
# then gets "...\obj_dir\Usersryazanovoss-cad-suiteshareverilator\bin\verilator_includer"
# and leaves an empty V<top>__ALL.cpp behind (fpga/sim/run.ps1 sidesteps the same
# thing with VM_PARALLEL_BUILDS=1). Windows and python take C:/... as they are.
$env:VERILATOR_ROOT = "$OssRoot\share\verilator" -replace '\\', '/'

# 2. Locate GCC / G++ (Must be GCC 13-14, GCC 16 breaks Verilator linking)
$gxx = $null
$wingetPkg = Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Recurse -Filter g++.exe -ErrorAction SilentlyContinue |
    Select-Object -First 1 -ExpandProperty FullName
if ($wingetPkg) {
    $gxx = $wingetPkg
} else {
    $cmd = Get-Command g++.exe -ErrorAction SilentlyContinue
    if ($cmd) { $gxx = $cmd.Source }
}

if (-not $gxx -or -not (Test-Path $gxx)) {
    throw ("G++ not found. Install GCC 14.2.0 via:`n" +
           "winget install --id BrechtSanders.WinLibs.POSIX.UCRT -v `"14.2.0-12.0.0-r2`" -e --scope user")
}

$gxxVersion = (& $gxx -dumpfullversion 2>$null)
if (-not $gxxVersion) { $gxxVersion = (& $gxx -dumpversion 2>$null) }
if ($gxxVersion -match '^1[56]\.') {
    Write-Warning ("Found GCC $gxxVersion. GCC >= 15 is incompatible with Verilator 5.053 " +
                  "(fails with undefined reference to basic_string in verilated.o). " +
                  "Downgrade to 14.2.0 using winget install --id BrechtSanders.WinLibs.POSIX.UCRT -v `"14.2.0-12.0.0-r2`" -e")
}

# 2a. make and a POSIX sh. --binary implies --build, which runs $env:MAKE (or
# plain `make`), and the generated makefile needs a real sh.exe - without one
# mingw32-make hands the recipes to cmd.exe ("0 was unexpected at this time").
# Neither oss-cad-suite nor Windows ships either. WinLibs puts mingw32-make next
# to g++; sh comes from a separate MSYS2 (setup.md). AndeSight's cygwin is
# deliberately NOT used: it dies on start with `add_item ... errno 1` whenever a
# stray cygwin process hangs (ERRORS 5.3), and since 06.10.2026 it is not on
# the user PATH at all.
$gxxDir = Split-Path $gxx
$mk = Join-Path $gxxDir 'mingw32-make.exe'
if (-not (Test-Path $mk)) { throw "mingw32-make.exe not found next to $gxx" }
$env:MAKE = $mk

$posix = $null
foreach ($r in @($env:MSYS2_ROOT, (Join-Path $env:LOCALAPPDATA 'msys64'), 'C:\msys64') | Where-Object { $_ }) {
    $b = Join-Path $r 'usr\bin'
    if ((Test-Path (Join-Path $b 'sh.exe')) -and (Test-Path (Join-Path $b 'uname.exe'))) { $posix = $b; break }
}
if (-not $posix) {
    throw ("No MSYS2 sh.exe/uname.exe found (MSYS2_ROOT, %LOCALAPPDATA%\msys64, C:\msys64). Install:`n" +
           "winget install --id MSYS2.MSYS2 -e --scope user --location `"$env:LOCALAPPDATA\msys64`"")
}

# oss-cad-suite FIRST (its DLLs), then the rest of PATH minus AndeSight (its
# cygwin would otherwise shadow sh/uname), MinGW and MSYS2 LAST so their
# same-named DLLs shadow nothing. Same order as fpga/sim/run.ps1.
$keep = $env:Path -split ';' | Where-Object { $_ -and $_ -notmatch 'Andestech' }
$env:Path = (@("$OssRoot\bin", "$OssRoot\lib") + $keep + @($gxxDir, $posix)) -join ';'

# 3. Resolve source files
$files = @(
    $Sources |
        ForEach-Object { $_ -split ',' } |
        Where-Object   { $_.Trim() } |
        ForEach-Object { Get-ChildItem -Path $_.Trim() -File | ForEach-Object { $_.FullName } }
)
if (-not $files) { throw "No source files matched: $($Sources -join ' ')" }

if ($CppMain) {
    if (-not (Test-Path $CppMain)) { throw "C++ testbench file not found: $CppMain" }
    $CppMain = (Get-Item $CppMain).FullName
}

# 4. Determine space-free build directory
if (-not $BuildDir) {
    $BuildDir = Join-Path "$env:USERPROFILE\.cache" ("vsim_" + $Top + "_" + [guid]::NewGuid().ToString('N').Substring(0, 6))
}
if ($BuildDir.Contains(' ')) {
    throw "BuildDir contains spaces ($BuildDir). Verilator GNU Make requires a path without spaces."
}
New-Item -ItemType Directory -Path $BuildDir -Force | Out-Null
$objDir = Join-Path $BuildDir "obj_dir"

try {
    # 5. Build argument list
    $waivers = $Waive | ForEach-Object { "-Wno-$_" }
    if ($BlackboxMissing) { $waivers += @('-Wno-MODMISSING', '-Wno-UNDRIVEN') }

    $argList = @(
        '--binary',
        '-Mdir', $objDir,
        '--prefix', "V$Top",
        '--top-module', $Top
    )
    if ($Timing) { $argList += '--timing' }
    if ($Trace)  { $argList += '--trace' }
    $argList += $waivers
    if ($ExtraArgs) { $argList += $ExtraArgs }
    $argList += $files
    if ($CppMain) { $argList += $CppMain }

    $err = Join-Path $BuildDir 'build_err.txt'
    $out = Join-Path $BuildDir 'build_out.txt'

    Write-Host "[verilator] Compiling $Top with GCC $gxxVersion..." -ForegroundColor Cyan
    $p = Start-Process -FilePath "$OssRoot\bin\verilator_bin.exe" -ArgumentList $argList `
                       -NoNewWindow -Wait -PassThru `
                       -RedirectStandardError $err -RedirectStandardOutput $out

    if ($p.ExitCode -ne 0) {
        if (Test-Path $err) { Get-Content $err | ForEach-Object { Write-Host $_ -ForegroundColor Red } }
        throw "Verilator build failed with exit code $($p.ExitCode)"
    }

    $exe = Join-Path $objDir "V$Top.exe"
    if (-not (Test-Path $exe)) {
        throw "Build succeeded but output binary not found: $exe"
    }

    Write-Host "[verilator] Build successful: $exe" -ForegroundColor Green

    # 6. Execute simulation
    if (-not $NoRun) {
        Write-Host "[verilator] Running simulation..." -ForegroundColor Cyan
        & $exe $RunArgs
        $simExit = $LASTEXITCODE
        Write-Host "[verilator] Simulation finished with exit code $simExit" `
                   -ForegroundColor $(if ($simExit -eq 0) { 'Green' } else { 'Red' })
        exit $simExit
    }
}
finally {
    # Clean up build artifacts if in temp cache
    if ($BuildDir -like "*\.cache\vsim_*") {
        Remove-Item -Recurse -Force $BuildDir -ErrorAction SilentlyContinue
    }
}
