# Power-cycle acceptance (AXI4_MIGRATION_PLAN.md, stage 3 -> 6): watches the UART
# log for every new boot ("[BOOT] Andes AE350 SoC booting...") and checks it the
# way a bitstream is accepted here: WD cmp growth over 20 s of idle (a sick boot,
# ERRORS 3.7), ping, sync_check (14 checks), one stereo pair (CRC, trailers) and
# 5 s of two-camera profile stream at 900 Hz (profiles skipped). One result line
# per boot goes to $Out; a system sound marks "done - power may be cut".
# Someone else toggles the power; this script only watches.
# The pair comes over UDP from the FPGA since 14.10.2026 (0x0224, stp_client
# --fetch); "TCP fallback" in the line means the board could not.
#   .\power_cycle_watch.ps1 -Log <uart.log> -Out <results.txt> -N 10
param(
    [string] $Log = "$PSScriptRoot\..\tools\uart\uart.log",
    [string] $Out = "$PSScriptRoot\power_cycles.txt",
    [int]    $N = 10,
    [int]    $Minutes = 120,
    [string] $Since = (Get-Date).ToString('HH:mm:ss'),
    # board address: vmsp.HOST (10.0.0.33 since 03.11.2026; 192.168.31.125 from 13.10.2026; 10.0.0.125 before)
    [string] $BoardIp = $(if ($env:VMSP_HOST) { $env:VMSP_HOST } else { '10.0.0.33' })
)
$repo = (Resolve-Path "$PSScriptRoot\..").Path
$env:PYTHONIOENCODING = 'utf-8'

# Only lines appended after the start count: the log carries times without a
# date, and on 12.10.2026 a log kept since 09.10 made this script take that
# afternoon's boots (14:21, 14:22 ...) for new ones and test the board against them.
$StartLine = @(Get-Content $Log -ErrorAction SilentlyContinue).Count
function Lines-Since([string] $t) {
    Get-Content $Log | Select-Object -Skip $StartLine |
        Where-Object { $_ -match '^\[(\d\d:\d\d:\d\d)' -and $Matches[1] -ge $t }
}
function Wd([string] $t) {
    $l = Lines-Since $t | Where-Object { $_ -match 'ETH STATS' } | Select-Object -Last 1
    if ($l -match 'WD cmp=(\d+)') { [int]$Matches[1] } else { -1 }
}
function Run-Py([string[]] $argv, [int] $timeout) {
    # VMSP_HOST: sync_check and profile_stream take the board address from it
    $j = Start-Job { param($repo, $argv, $ip) Set-Location $repo; $env:PYTHONIOENCODING = 'utf-8'; $env:VMSP_HOST = $ip
                     [Console]::OutputEncoding = [Text.Encoding]::UTF8; python @argv 2>&1 } -ArgumentList $repo, $argv, $BoardIp
    $o = if (Wait-Job $j -Timeout $timeout) { (Receive-Job $j) -join "`n" } else { Stop-Job $j; 'TIMEOUT' }
    Remove-Job $j -Force
    $o
}

"# power-cycle acceptance, started $(Get-Date -Format 'dd.MM.yyyy HH:mm:ss'), log $Log" | Add-Content $Out
$deadline = (Get-Date).AddMinutes($Minutes)
$done = 0
$seen = $Since
while ($done -lt $N -and (Get-Date) -lt $deadline) {
    Start-Sleep 2
    $boot = Lines-Since $seen | Where-Object { $_ -match 'BOOT\] Andes AE350 SoC booting' } | Select-Object -First 1
    if (-not $boot) { continue }
    $tb = $boot.Substring(1, 8)
    # next search starts a second after this boot line: a boot during the tests is
    # then found (and flagged) on the next round instead of being skipped
    $seen = [datetime]::ParseExact($tb, 'HH:mm:ss', $null).AddSeconds(1).ToString('HH:mm:ss')
    # network up?
    $up = $false
    for ($k = 0; $k -lt 60 -and -not $up; $k++) {
        Start-Sleep 1
        $up = [bool](Lines-Since $tb | Where-Object { $_ -match 'TCP control listening' })
    }
    $done++
    if (-not $up) { "{0,2} boot {1}: NO NETWORK within 60 s" -f $done, $tb | Add-Content $Out; continue }
    $cams = (Lines-Since $tb | Where-Object { $_ -match 'DETECTED' }).Count
    Start-Sleep 5
    $w0 = Wd $tb; Start-Sleep 20; $w1 = Wd $tb
    $ping = (ping -n 10 -w 1000 $BoardIp | Select-String 'Received = (\d+)').Matches[0].Groups[1].Value
    $sc = Run-Py @('tools/camera/sync_check.py') 300
    $scr = if ($sc -match 'итого: (\d+/\d+)') { $Matches[1] } else { 'no result' }
    $st = Run-Py @('tools/capture/stp_client.py', '--host', $BoardIp, '--camera-id', 'stereo', '--acquire', '1', '--fetch', '1') 120
    $crc = ([regex]::Matches($st, 'CRC ok')).Count
    $badr = ([regex]::Matches($st, 'плохим трейлером ([1-9]\d*)')).Count
    $via = if ($st -match 'кадры по TCP') { 'TCP fallback' } else { 'UDP' }
    $ps = Run-Py @('tools/laser/profile_stream.py', '--cams', '0', '1', '--freq', '900', '--seconds', '5') 120
    $m = [regex]::Match($ps, 'отдано \[(\d+), (\d+)\], пропущено \[(\d+), (\d+)\]')
    $prof = if ($m.Success) { "profiles {0}/{1} skipped {2}/{3}" -f $m.Groups[1].Value, $m.Groups[2].Value, $m.Groups[3].Value, $m.Groups[4].Value } else { 'profiles: no totals' }
    $rebooted = [bool](Lines-Since $seen | Where-Object { $_ -match 'BOOT\] Andes AE350 SoC booting' })
    $w2 = Wd $tb
    ("{0,2} boot {1}: cams {2}, WD cmp {3}->{4} ({5}), {6} after tests; ping {7}/10; sync_check {8}; stereo CRC ok {9}/2 ({13}), bad-trailer frames {10}; {11}{12}" -f
        $done, $tb, $cams, $w0, $w1, $(if ($w1 -gt $w0) { 'SICK' } else { 'healthy' }), $w2, $ping, $scr, $crc, $badr, $prof,
        $(if ($rebooted) { ' [power cut during the tests]' } else { '' }), $via) | Add-Content $Out
    try { [System.Media.SystemSounds]::Exclamation.Play() } catch {}
}
"# finished $(Get-Date -Format 'HH:mm:ss'): $done boots" | Add-Content $Out
