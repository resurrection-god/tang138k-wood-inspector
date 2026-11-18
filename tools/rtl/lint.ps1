<#
.SYNOPSIS
    Verilator lint on Windows, with the plumbing that makes its output visible.

.DESCRIPTION
    Three things silently defeat a naive `verilator --lint-only` on Windows, and
    all three look like a clean lint:

      * VERILATOR_ROOT unset  -> "Cannot find verilated_std_waiver.vlt"
      * lib missing from PATH -> exit -1073741515 (DLL not found), no output
      * findings go to stderr -> PowerShell's >, 2>&1 and pipelines discard them

    This wrapper sets the first two and uses Start-Process -RedirectStandardError,
    which is the only redirection form that actually captures Verilator's output.

    Exit code is the finding count, so a caller can gate on it. -Mutate proves the
    harness can fail before you trust a clean run.

.EXAMPLE
    .\lint.ps1 -Top my_module -Sources src\*.v

.EXAMPLE
    # Gate: corrupt a signal width and confirm the lint notices.
    .\lint.ps1 -Top my_module -Sources src\*.v -Mutate 'reg [31:0] addr;=reg [7:0] addr;'
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]   $Top,
    [Parameter(Mandatory)][string[]] $Sources,
    [string]   $OssRoot = "$env:USERPROFILE\oss-cad-suite",
    # PINCONNECTEMPTY is in the default set because any design that instantiates a
    # vendor macro leaves outputs unconnected on purpose, and dozens of them drown
    # the real findings - including in the control mutant, which silently destroys
    # the gate. Drop it from -Waive if you want to audit unconnected pins.
    [string[]] $Waive   = @('DECLFILENAME', 'UNUSEDSIGNAL', 'UNUSEDPARAM',
                            'VARHIDDEN', 'PINCONNECTEMPTY'),
    # Blackbox vendor primitives that have no source (encrypted / IDE-generated).
    # UNDRIVEN comes along with it: a blackboxed cell drives nothing.
    [switch]   $BlackboxMissing,
    # 'find=replace' applied to the first source file, in a scratch copy.
    [string]   $Mutate
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path "$OssRoot\bin\verilator_bin.exe")) {
    throw "verilator_bin.exe not found under $OssRoot. Pass -OssRoot."
}

$env:VERILATOR_ROOT = "$OssRoot\share\verilator"
$env:Path           = "$OssRoot\bin;$OssRoot\lib;" + $env:Path

# `powershell -File script.ps1 -Sources a,b,c` hands the array over as ONE
# comma-joined string - a documented wart of -File, and the way an automated
# caller will most often invoke this. Split it back, and expand wildcards, so
# both invocation styles work.
$files = @(
    $Sources |
        ForEach-Object { $_ -split ',' } |
        Where-Object   { $_.Trim() } |
        ForEach-Object { Get-ChildItem -Path $_.Trim() -File | ForEach-Object { $_.FullName } }
)
if (-not $files) { throw "No source files matched: $($Sources -join ' ')" }
$work  = Join-Path ([System.IO.Path]::GetTempPath()) ("lint_" + [guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory -Path $work -Force | Out-Null

try {
    if ($Mutate) {
        $find, $replace = $Mutate -split '=', 2
        $copy = Join-Path $work (Split-Path $files[0] -Leaf)
        $text = [System.IO.File]::ReadAllText($files[0], [System.Text.Encoding]::UTF8)
        if (-not $text.Contains($find)) { throw "Mutation target not found: $find" }
        [System.IO.File]::WriteAllText($copy, $text.Replace($find, $replace),
                                       (New-Object System.Text.UTF8Encoding($false)))
        $files[0] = $copy
        Write-Host "[mutant] $find  ->  $replace" -ForegroundColor Yellow
    }

    $waivers = $Waive | ForEach-Object { "-Wno-$_" }
    if ($BlackboxMissing) { $waivers += @('-Wno-MODMISSING', '-Wno-UNDRIVEN') }

    $argList = @('--lint-only', '-Wall') + $waivers + @('--top-module', $Top) + $files
    $err = Join-Path $work 'err.txt'
    $out = Join-Path $work 'out.txt'

    $p = Start-Process -FilePath "$OssRoot\bin\verilator_bin.exe" -ArgumentList $argList `
                       -NoNewWindow -Wait -PassThru `
                       -RedirectStandardError $err -RedirectStandardOutput $out

    $findings = @(Get-Content $err -ErrorAction SilentlyContinue |
                  Select-String -Pattern '^%(Warning|Error)')
    $findings | ForEach-Object { $_.Line }

    if ($findings.Count -eq 0 -and $p.ExitCode -ne 0) {
        Write-Warning ("verilator exited $($p.ExitCode) but produced no parsable findings. " +
                       "That is the silent-failure case - check VERILATOR_ROOT and PATH.")
    }
    Write-Host "findings: $($findings.Count)  (verilator exit $($p.ExitCode))" `
               -ForegroundColor $(if ($findings.Count) { 'Red' } else { 'Green' })
    exit $findings.Count
}
finally {
    Remove-Item $work -Recurse -Force -ErrorAction SilentlyContinue
}
