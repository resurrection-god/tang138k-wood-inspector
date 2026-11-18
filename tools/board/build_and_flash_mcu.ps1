<#
.SYNOPSIS
    Builds and flashes the Andes RISC-V C firmware to Tang Mega 138K Pro SPI Flash (0x600000).

.DESCRIPTION
    Environment the AndeSight make needs:
      * cygwin\bin, toolchain\bin AND AndeSight_RDS_v511\utils on PATH. utils holds
        nds_ldsag; an incremental build never calls it, a clean one dies with
        "nds_ldsag: command not found" after make clean already deleted the .bin.
        cygwin\bin is not on the user PATH since 06.10.2026 (ERRORS 5.11), so this
        script must put it there: make and the toolchain load cygwin1.dll from it.
      * HOME and SHELL - harmless, but not what prevents
        "add_item (...cygwin, /, ...) failed, errno 1" (refuted 06.10.2026). That
        failure is a hung cygwin ps.exe (Orca's terminal daemon, ERRORS 5.11): while
        it hangs every new cygwin process dies at start. The build is still retried
        up to three times on that message only.

    The build is byte-for-byte reproducible, so the SHA-256 printed at the end is a
    real identity of what gets flashed.
#>

[CmdletBinding()]
param(
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

$scriptDir     = Split-Path -Parent $MyInvocation.MyCommand.Path
$workspaceRoot = (Resolve-Path "$scriptDir\..\..").Path
$mcuDebugDir   = "$workspaceRoot\firmware\Debug"
$mcuBinFile    = "$mcuDebugDir\output\ae350_scanner.bin"
$andes         = "C:\Andestech\AndeSight_RDS_v511"
$makeExe       = "$andes\cygwin\bin\make.exe"
$progCli       = "C:\Gowin\Gowin_V1.9.12.04_x64\Programmer\bin\programmer_cli.exe"
if (-not (Test-Path $progCli)) {
    $progCli   = "C:\Gowin\Gowin_V1.9.12.03_x64\Programmer\bin\programmer_cli.exe"
}

Write-Host "=======================================================" -ForegroundColor Cyan
Write-Host " [MCU FLASH] Tang Mega 138K Pro Andes AE350 Firmware" -ForegroundColor Cyan
Write-Host "=======================================================" -ForegroundColor Cyan

if (-not $SkipBuild) {
    Write-Host "[1/2] Compiling RISC-V C Firmware..." -ForegroundColor Yellow
    $env:Path  = "$andes\cygwin\bin;$andes\toolchains\nds32le-elf-mculib-v5\bin;$andes\utils;" + $env:Path
    $env:HOME  = "$andes\ice"
    $env:SHELL = "/bin/bash"

    Push-Location $mcuDebugDir
    try {
        $ok = $false
        for ($attempt = 1; $attempt -le 3 -and -not $ok; $attempt++) {
            $out = & $makeExe all 2>&1 | ForEach-Object { $_.ToString() }
            $rc  = $LASTEXITCODE
            if ($rc -eq 0) { $ok = $true; break }
            if (($out -join "`n") -match 'add_item') {
                Write-Host "      cygwin add_item glitch (attempt $attempt), retrying..." -ForegroundColor DarkYellow
                continue
            }
            $out | Select-Object -Last 30 | ForEach-Object { Write-Host $_ }
            Write-Error "Firmware build failed with exit code $rc"
        }
        if (-not $ok) { Write-Error "Firmware build failed: cygwin add_item error on every attempt" }
    } finally {
        Pop-Location
    }
} else {
    Write-Host "[1/2] Skipping build step (-SkipBuild specified)." -ForegroundColor Gray
}

if (-not (Test-Path $mcuBinFile)) {
    Write-Error "Binary output file not found: $mcuBinFile"
}
Write-Host ("      ae350_scanner.bin SHA-256 " + (Get-FileHash $mcuBinFile -Algorithm SHA256).Hash)

# programmer_cli embeds an old Python that dies on this variable
# ("unknown encoding: utf-8:surrogateescape"), and a running ICEman holds the
# FT2232H and turns a 5 s write into minutes (ch. 31.4).
Remove-Item Env:\PYTHONIOENCODING -ErrorAction SilentlyContinue
Get-Process ICEman, riscv32-elf-gdb -ErrorAction SilentlyContinue | Stop-Process -Force -Confirm:$false

Write-Host "[2/2] Flashing MCU Binary to SPI Flash (addr: 0x600000, freq: 15MHz)..." -ForegroundColor Yellow
& $progCli --device "GW5AST-138B" --freq 15MHz --run 55 --mcuFile $mcuBinFile --spiaddr 0x600000

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n[SUCCESS] MCU Firmware successfully flashed to SPI Flash (0x600000)!" -ForegroundColor Green
} else {
    Write-Error "Programmer CLI failed with exit code $LASTEXITCODE"
}
