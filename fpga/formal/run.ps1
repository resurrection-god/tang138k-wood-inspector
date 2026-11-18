<#
.SYNOPSIS
    Runs the SymbiYosys proofs in fpga/formal with OSS CAD Suite on Windows.

.DESCRIPTION
    .\run.ps1 <job> [task ...]   e.g.  .\run.ps1 cam_axi_wr prove cover
    .\run.ps1 all                every *.sby here, every task

    Work directories go to $env:TEMP\fpga_formal\<job>_<task> (sby --prefix),
    not into the repository. Prints one line per task: PASS / FAIL / UNKNOWN /
    ERROR, and the path of the log and of the counterexample trace on a failure.
    Exit code = number of tasks that did not pass.

    Every proof here has been shown to catch a mutant (README.md); a PASS of a
    property file nobody has seen fail means nothing (WORKFLOWS.md section 10).
#>
param(
    [Parameter(Mandatory = $true, Position = 0)] [string]   $Job,
    [Parameter(Position = 1, ValueFromRemainingArguments = $true)] [string[]] $Tasks,
    [string] $OssRoot = "$env:USERPROFILE\oss-cad-suite",
    [string] $WorkRoot = "$env:TEMP\fpga_formal"
)

$ErrorActionPreference = 'Stop'
if (-not (Test-Path "$OssRoot\bin\sby.exe")) { throw "sby.exe not found under $OssRoot. Pass -OssRoot." }
$env:YOSYSHQ_ROOT      = "$OssRoot\"
$env:PATH              = "$OssRoot\bin;$OssRoot\lib;$env:PATH"
$env:PYTHON_EXECUTABLE = "$OssRoot\lib\python3.exe"
Remove-Item Env:PYTHONIOENCODING -ErrorAction SilentlyContinue

Push-Location $PSScriptRoot
try {
    $jobs = if ($Job -eq 'all') { (Get-ChildItem *.sby).BaseName } else { @($Job) }
    New-Item -ItemType Directory -Force $WorkRoot | Out-Null
    $bad = 0
    foreach ($j in $jobs) {
        $sbyFile = "$j.sby"
        if (-not (Test-Path $sbyFile)) { throw "$sbyFile not found in $PSScriptRoot" }
        $list = $Tasks
        if (-not $list -or $Job -eq 'all') {
            $text = Get-Content $sbyFile -Raw
            if ($text -match '(?ms)^\[tasks\]\s*\r?\n(.*?)(\r?\n\[|\z)') {
                $list = $Matches[1] -split '\r?\n' | ForEach-Object { ($_ -replace '#.*', '').Trim() } |
                        Where-Object { $_ } | ForEach-Object { ($_ -split '\s+')[0] }
            } else { $list = @('') }
        }
        foreach ($t in $list) {
            # sby appends _<task> to --prefix itself
            $name   = if ($t) { "${j}_$t" } else { $j }
            $prefix = Join-Path $WorkRoot $name
            $args_  = @('-f', '--prefix', (Join-Path $WorkRoot $j), $sbyFile)
            if ($t) { $args_ += $t }
            $sw = [Diagnostics.Stopwatch]::StartNew()
            $out = & "$OssRoot\bin\sby.exe" @args_ 2>&1 | Out-String
            $sw.Stop()
            $status = if (Test-Path "$prefix\status") { (Get-Content "$prefix\status" -Raw).Trim() } else { 'ERROR (no status file)' }
            $line = '{0,-28} {1,-30} {2,6:N1} s' -f $name, $status, $sw.Elapsed.TotalSeconds
            if ($status -match '^PASS') {
                Write-Host $line -ForegroundColor Green
            } else {
                $bad++
                Write-Host $line -ForegroundColor Red
                Write-Host "    log: $prefix\logfile.txt"
                Get-ChildItem $prefix -Recurse -Filter *.vcd -ErrorAction SilentlyContinue |
                    Select-Object -First 3 | ForEach-Object { Write-Host "    vcd: $($_.FullName)" }
                ($out -split '\r?\n') | Where-Object { $_ -match '(?i)assert|fail|error|unknown' } |
                    Select-Object -First 12 | ForEach-Object { Write-Host "    $_" }
            }
        }
    }
    exit $bad
} finally {
    Pop-Location
}
