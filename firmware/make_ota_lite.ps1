# Lightweight Bootloader Verification Firmware build (-O2 -DOTA_LITE_PROFILE=1)
# Output: firmware\Release_Lite\output\ae350_ota_lite.bin
$ErrorActionPreference = 'Stop'
$fw = $PSScriptRoot
$lite = Join-Path $fw 'Release_Lite'

if (Test-Path $lite) { Remove-Item -Recurse -Force $lite }
robocopy (Join-Path $fw 'Debug') $lite /E /XF *.o *.d *.adx *.bin /NFL /NDL /NJH /NJS | Out-Null

Get-ChildItem $lite -Recurse -Include *.mk, makefile | ForEach-Object {
    $t = [IO.File]::ReadAllText($_.FullName)
    $n = $t.Replace(' -Og ', ' -O2 -DOTA_LITE_PROFILE=1 ').Replace(' -g3 ', ' -g ')
    $n = $n.Replace('ae350_scanner', 'ae350_ota_lite')
    if ($n -ne $t) { [IO.File]::WriteAllText($_.FullName, $n) }
}

$subdir = Join-Path $lite 'src\app\subdir.mk'
$lines = Get-Content $subdir
$drop = @('camera.', 'cam_trig.', 'console.', 'frame_udp.', 'gev_xml.', 'gvcp.', 'gvsp.',
          'gvsp_gate.', 'hik_client.', 'i2c_bus.', 'laser.', 'mailbox.', 'mm_table.', 'settings.', 'sync.',
          'video.', 'work_mode.')
$out = foreach ($l in $lines) {
    $skip = $false
    foreach ($d in $drop) {
        if ($l -match [regex]::Escape($d)) { $skip = $true; break }
    }
    if (-not $skip) {
        $l.Replace('vmsp_service.', 'ota_lite_service.')
    }
}
Set-Content -Path $subdir -Value $out -Encoding UTF8

Push-Location $lite
try {
    & 'C:\Andestech\AndeSight_RDS_v511\cygwin\bin\make.exe' -j4 all
    if ($LASTEXITCODE -ne 0) { throw "make failed: $LASTEXITCODE" }
} finally { Pop-Location }
