# Boot series for comparing the AXI back end with the production bitstream
# (AXI4_MIGRATION_PLAN.md stage 3): N boots, each judged by WD cmp growth over
# 20 s of idle (a sick boot - ERRORS 3.7/3.10 - grows from the start), ping, and
# 5 s of two-camera profile stream (profiles the board skipped).
#   -Mode axi  : load $Fs into SRAM (--run 2)
#   -Mode prod : reload the bitstream in the SPI flash (--run 1)
param(
    [ValidateSet('axi', 'prod')] [string] $Mode = 'axi',
    [int]    $N   = 4,
    [string] $Fs  = "$PSScriptRoot\impl\pnr\scanner.fs",
    [string] $Log = "$PSScriptRoot\..\tools\uart\uart.log",  # uart_logger.py --logfile
    # board address: vmsp.HOST (10.0.0.33 since 03.11.2026; 192.168.31.125 from 13.10.2026; 10.0.0.125 before)
    [string] $BoardIp = $(if ($env:VMSP_HOST) { $env:VMSP_HOST } else { '10.0.0.33' })
)
Remove-Item Env:\PYTHONIOENCODING -ErrorAction SilentlyContinue
$PGM  = "C:\Gowin\Gowin_V1.9.12.03_x64\Programmer\bin\programmer_cli.exe"
$repo = (Resolve-Path "$PSScriptRoot\..").Path

function Get-WdCmp([string] $since) {
    $l = Get-Content $Log | Where-Object { $_ -match '^\[(\d\d:\d\d:\d\d)' -and $Matches[1] -ge $since -and $_ -match 'ETH STATS' } | Select-Object -Last 1
    if ($l -match 'WD cmp=(\d+)') { [int]$Matches[1] } else { $null }
}

$rows = @()
for ($i = 1; $i -le $N; $i++) {
    $t = (Get-Date).ToString('HH:mm:ss')
    $j = if ($Mode -eq 'axi') {
        Start-Job { param($PGM, $fs) & $PGM --device GW5AST-138B --freq 15MHz --run 2 --fsFile $fs --cable-index 1 --location 0 2>&1 } -ArgumentList $PGM, $Fs
    } else {
        Start-Job { param($PGM) & $PGM --device GW5AST-138B --run 1 --cable-index 1 --location 0 2>&1 } -ArgumentList $PGM
    }
    $ok = Wait-Job $j -Timeout 180
    $out = if ($ok) { (Receive-Job $j) -join ' ' } else { Stop-Job $j; 'TIMEOUT' }
    Remove-Job $j -Force
    # wait for the network to come up
    $up = $false
    for ($k = 0; $k -lt 60 -and -not $up; $k++) {
        Start-Sleep 1
        $up = [bool](Get-Content $Log | Where-Object { $_ -match '^\[(\d\d:\d\d:\d\d)' -and $Matches[1] -ge $t -and $_ -match 'TCP control listening' })
    }
    if (-not $up) { $rows += "{0} {1,2}  no boot within 60 s ({2})" -f $Mode, $i, $out.Substring(0, [Math]::Min(80, $out.Length)); continue }
    Start-Sleep 5
    $w0 = Get-WdCmp $t
    Start-Sleep 20
    $w1 = Get-WdCmp $t
    $ping = (ping -n 10 -w 1000 $BoardIp | Select-String 'Received = (\d+)').Matches[0].Groups[1].Value
    $pj = Start-Job { param($repo, $ip) Set-Location $repo; $env:PYTHONIOENCODING = 'utf-8'; $env:VMSP_HOST = $ip; python tools/laser/profile_stream.py --cams 0 1 --freq 900 --seconds 5 2>&1 } -ArgumentList $repo, $BoardIp
    $pout = if (Wait-Job $pj -Timeout 120) { (Receive-Job $pj) -join "`n" } else { Stop-Job $pj; 'TIMEOUT' }
    Remove-Job $pj -Force
    $m = [regex]::Match($pout, '\[(\d+), (\d+)\][^\[\n]*\[(\d+), (\d+)\]')
    $prof = if ($m.Success) { "profiles sent {0}/{1}, skipped {2}/{3}" -f $m.Groups[1].Value, $m.Groups[2].Value, $m.Groups[3].Value, $m.Groups[4].Value } else { "profile stream: no totals" }
    $w2 = Get-WdCmp $t
    $rows += "{0} {1,2}  WD cmp {2} -> {3} in 20 s idle ({4}), {5} after stream; ping {6}/10; {7}" -f $Mode, $i, $w0, $w1,
             $(if ($w1 -gt $w0) { 'SICK' } else { 'healthy' }), $w2, $ping, $prof
}
$rows
