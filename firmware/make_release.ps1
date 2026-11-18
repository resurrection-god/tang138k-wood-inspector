# Release build of the firmware (-O2 -g instead of -Og -g3), the one flashed since 22.10.2026
# (OPEN_ISSUES 3.1). firmware/Debug stays the debug build; firmware/Release is generated from its
# AndeSight makefiles (both untracked) with only the optimisation flags changed, then built.
#   .\firmware\make_release.ps1            # -> firmware\Release\output\ae350_scanner.bin
# Needs the same PATH as the Debug build (docs/WORKFLOWS.md § 4) and the sandbox off.
$ErrorActionPreference = 'Stop'
$andes = 'C:\Andestech\AndeSight_RDS_v511'
if (Test-Path $andes) {
    $env:Path = "$andes\cygwin\bin;$andes\toolchains\nds32le-elf-mculib-v5\bin;$andes\utils;" + $env:Path
    $env:HOME = "$andes\ice"
    $env:SHELL = "/bin/bash"
}
$fw = $PSScriptRoot
$rel = Join-Path $fw 'Release'
if (Test-Path $rel) { Remove-Item -Recurse -Force $rel }
Get-ChildItem (Join-Path $fw 'Debug') -Recurse -Include *.mk, makefile | ForEach-Object {
    $t = [IO.File]::ReadAllText($_.FullName)
    $n = [regex]::Replace($t, '/cygdrive/[A-Za-z]/[^"\s]+/firmware', '..')
    if ($n -ne $t) { [IO.File]::WriteAllText($_.FullName, $n) }
}
robocopy (Join-Path $fw 'Debug') $rel /E /XF *.o *.d *.adx *.bin /NFL /NDL /NJH /NJS | Out-Null
Get-ChildItem $rel -Recurse -Include *.mk, makefile | ForEach-Object {
    $t = [IO.File]::ReadAllText($_.FullName)
    $n = $t.Replace(' -Og ', ' -Os -mno-execit ').Replace(' -g3 ', ' -g ')
    if ($n -ne $t) { [IO.File]::WriteAllText($_.FullName, $n) }
}
Push-Location $rel
try {
    & 'C:\Andestech\AndeSight_RDS_v511\cygwin\bin\make.exe' -j4 all
    if ($LASTEXITCODE -ne 0) { throw "make failed: $LASTEXITCODE" }
} finally { Pop-Location }
