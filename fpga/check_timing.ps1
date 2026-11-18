<#
.SYNOPSIS
    Post-PnR timing gate: every clock must reach its constraint.

.DESCRIPTION
    CLAUDE.md makes this check mandatory before flashing, and it has been done by
    eye - opening scanner_tr_content.html and reading a table. That is
    exactly how a build with ae350_ahb_clk at 95.378 MHz against a 100 MHz
    constraint reached the board and looked like a memory-calibration problem
    (ch. 31.8).

    Reads the PnR timing report, prints Actual Fmax against Constraint for every
    clock, and exits with the number of violations so a caller can gate on it.

    ae350_ahb_clk deserves special attention: it has effectively zero margin and
    closes somewhere between 99.6 and 100.4 MHz depending on how the netlist
    happens to place (ch. 28.5), so a small miss there is a placement lottery
    while a large one means the logic actually got deeper.

    Also gates on what the Fmax table cannot show (25.09.2026): recovery,
    removal and hold slack from the report's path sections, the report's own
    "Numbers of ... Violated Endpoints" lines, and a margin on the clocks whose
    logic is ours. Build 17FDDAC6 passed the old version of this script with
    exit 0 - every Fmax PASS, worst setup path +0.014 ns - while the report said
    "Numbers of Setup Violated Endpoints 2" (recovery -2.781 ns on soc_rst_n ->
    lane4_rst_sync), and on the board it corrupted DDR3 (build_fpga.tcl, L2a).

.EXAMPLE
    .\check_timing.ps1
    if ($LASTEXITCODE -ne 0) { "do not flash" }

.EXAMPLE
    .\check_timing.ps1 -MinSlack 0     # hard violations only, no margin rule
#>
[CmdletBinding()]
param(
    # $PSScriptRoot, not $MyInvocation.MyCommand.Path: the latter is null while a
    # param() default is being evaluated under `powershell -File`, which is how a
    # caller most naturally runs this.
    [string] $Report,

    # Setup margin (ns) required on the clocks in -OwnClocks. A build at +0.014 ns
    # is a coin toss, and the board has twice shown faults the timing report does
    # not see (ERRORS 1.4, build_fpga.tcl L2a). The vendor-dominated clocks
    # (ae350_ahb_clk: the SoC's own SPI-flash FIFO -> ROM_HRDATA paths) cannot be
    # held to this and are gated at 0.
    [double] $MinSlack = 0.3,

    # clk_x1 (the DDR3 IP's clk_out: camera writers, axi_mux, adapters) replaced
    # lane4_clk with the AXI4 back end on 09.10.2026 (ERRORS 1.21).
    [string[]] $OwnClocks = @('clk_x1', 'mipi_byte_clk', 'mipi1_byte_clk')
)

if (-not $Report) {
    $Report = Join-Path $PSScriptRoot 'impl\pnr\scanner_tr_content.html'
}

$ErrorActionPreference = 'Stop'
if (-not (Test-Path $Report)) { throw "timing report not found: $Report" }

$raw = Get-Content $Report -Raw
$i = $raw.IndexOf('Actual Fmax')
if ($i -lt 0) { throw "no 'Actual Fmax' table in $Report - did PnR finish?" }

# The table is one <td> per cell; flatten to a list and walk it four at a time
# (clock, constraint, actual, entity).
$seg = $raw.Substring($i, [Math]::Min(12000, $raw.Length - $i))
$seg = $seg -replace '</td>', "`n" -replace '</tr>', "`n" -replace '<[^>]+>', '' -replace '&nbsp;', ' '
$cells = @($seg -split "`n" | ForEach-Object { $_.Trim() } | Where-Object { $_.Length -gt 0 })

$violations = 0
$rows = 0
# Do NOT walk the cells at a fixed stride. The row width is not stable - the
# report has carried 4 columns and 6 (an index and a logic-level column appear
# and disappear between runs), and a hard-coded stride silently parses zero rows
# or, worse, the wrong ones. Instead recognise the shape: a clock name is any
# cell followed by two that both carry "(MHz)".
for ($k = 0; $k -lt $cells.Count - 2; $k++) {
    $name = $cells[$k]
    $con  = $cells[$k + 1]
    $act  = $cells[$k + 2]
    if ($con -notmatch '^[\d.]+\(MHz\)$' -or $act -notmatch '^[\d.]+\(MHz\)$') { continue }
    if ($name -match '\(MHz\)' -or $name -match '^\d+$') { continue }

    $c = [double]($con -replace '\(MHz\)', '')
    $a = [double]($act -replace '\(MHz\)', '')
    $rows++
    # A hair of tolerance: the tool reports the constraint as e.g. 100.023 MHz
    # when the period is 9.998 ns, and a clock exactly at its constraint is fine.
    $ok = ($a -ge ($c - 0.001))
    if (-not $ok) { $violations++ }

    $margin = $a - $c
    # Setup slack of the clock's worst intra-clock path, from the same two numbers.
    $slackNs = 1000.0 / $c - 1000.0 / $a
    $own = $OwnClocks -contains $name
    $short = $ok -and $own -and ($slackNs -lt $MinSlack)
    if ($short) { $violations++ }
    $tag = if (-not $ok) { 'FAIL' } elseif ($short) { "MARGIN < $MinSlack ns" } else { 'PASS' }
    $col = if (-not $ok -or $short) { 'Red' } elseif ($margin -lt 1.0) { 'Yellow' } else { 'Green' }
    Write-Host ("  {0,-20} constraint {1,10:N3}  actual {2,10:N3}  margin {3,8:N3}  slack {4,7:N3} ns{5} {6}" `
                -f $name, $c, $a, $margin, $slackNs, $(if ($own) { '*' } else { ' ' }), $tag) -ForegroundColor $col
}
Write-Host "  (* = own logic, held to $MinSlack ns of setup slack; -MinSlack 0 turns the rule off)"

if ($rows -eq 0) { throw "parsed no clock rows - the report format may have changed" }

# The Fmax table is NOT the whole report, and trusting it alone is how a build
# with -5.4 ns of setup slack on the reset path into the pixel FIFO passed this
# check ten times over. That path is a cross-domain one and never appears as a
# clock's Fmax, so it has to be read from the path list separately.
$paths = Join-Path (Split-Path $Report) 'scanner.timing_paths'
if (Test-Path $paths) {
    # Format is blocks separated by "=====", each starting with the analysis type
    # and then the slack.
    $lines = @(Get-Content $paths)
    $worst = $null
    for ($i = 0; $i -lt $lines.Count - 2; $i++) {
        if ($lines[$i].Trim() -eq '=====') {
            $slack = 0.0
            if ([double]::TryParse($lines[$i + 2].Trim(),
                                   [Globalization.NumberStyles]::Float,
                                   [Globalization.CultureInfo]::InvariantCulture,
                                   [ref]$slack)) {
                if ($null -eq $worst -or $slack -lt $worst) { $worst = $slack }
            }
        }
    }
    if ($null -ne $worst) {
        Write-Host ""
        if ($worst -lt 0) {
            Write-Host ("  worst path slack     : {0,10:N3} ns  FAIL - a path misses setup" -f $worst) `
                       -ForegroundColor Red
            $violations++
        } else {
            Write-Host ("  worst path slack     : {0,10:N3} ns  PASS" -f $worst) -ForegroundColor Green
        }
    }
}

# Setup paths are all that scanner.timing_paths carries. Recovery, removal and
# hold live only in the HTML report's path sections ("report_timing -recovery
# ... Path1 Path Summary: Slack -2.781"), and the report also states its own
# count of violated endpoints - which counted the recovery paths of 17FDDAC6
# under "Setup" while every clock's TNS line read 0.000.
$flat = [regex]::Replace($raw, '<[^>]+>', ' ')
$flat = [regex]::Replace($flat, '&nbsp;', ' ')
$flat = [regex]::Replace($flat, '\s+', ' ')
Write-Host ""
foreach ($kind in 'hold', 'recovery', 'removal') {
    $ms = [regex]::Matches($flat, "report_timing -$kind\b.{0,400}?Path Summary: Slack (-?[\d.]+)")
    if ($ms.Count -eq 0) {
        Write-Host ("  worst {0,-9} slack : not in report" -f $kind) -ForegroundColor Yellow
        continue
    }
    $w = ($ms | ForEach-Object { [double]::Parse($_.Groups[1].Value, [Globalization.CultureInfo]::InvariantCulture) } |
          Measure-Object -Minimum).Minimum
    if ($w -lt 0) {
        $violations++
        Write-Host ("  worst {0,-9} slack : {1,10:N3} ns  FAIL" -f $kind, $w) -ForegroundColor Red
    } else {
        Write-Host ("  worst {0,-9} slack : {1,10:N3} ns  PASS" -f $kind, $w) -ForegroundColor Green
    }
}
# (Until 09.10.2026 the Lane 4 <-> DDR3 crossings ddr3_sysclk <-> lane4_clk were
# checked here, ERRORS 6.14. With the AXI4 back end there is no such crossing:
# writers, axi_mux and the IP's user side share clk_x1, and the stage-3 timing
# lesson - the mux-to-master paths - is covered by clk_x1's own-logic margin.)
# Every .sdc command must be "Actived" in the report's Timing Constraints
# Report (SUG940 4.2.8). An "Invalid" one - e.g. a get_pins path that a rename
# broke, as the camera-channel split to u_cam0/u_rx/... could have - silently
# constrains nothing, which is how a domain ends up unconstrained under a PASS.
$ci = $flat.LastIndexOf('Timing Constraints Report')
if ($ci -ge 0) {
    $seg = $flat.Substring($ci)
    $nAct = ([regex]::Matches($seg, '\bActived\b')).Count
    $inv  = [regex]::Matches($seg, '(\S+) Invalid (.{0,160})')
    if ($inv.Count -gt 0) {
        $violations++
        Write-Host ("  sdc commands        : {0} active, {1} INVALID  FAIL" -f $nAct, $inv.Count) -ForegroundColor Red
        foreach ($m in $inv) { Write-Host ("    {0} {1}" -f $m.Groups[1].Value, $m.Groups[2].Value) -ForegroundColor Red }
    } else {
        Write-Host ("  sdc commands        : {0} active, 0 invalid  PASS" -f $nAct) -ForegroundColor Green
    }
} else {
    Write-Host "  sdc commands        : no Timing Constraints Report found" -ForegroundColor Yellow
}
foreach ($m in [regex]::Matches($flat, 'Numbers of (\w+) Violated Endpoints (\d+)')) {
    $n = [int]$m.Groups[2].Value
    if ($n -gt 0) {
        $violations++
        Write-Host ("  report: {0} violated endpoints {1}  FAIL" -f $m.Groups[1].Value, $n) -ForegroundColor Red
    }
}

Write-Host ""
if ($violations -eq 0) {
    Write-Host "timing: $rows clocks, setup/hold/recovery/removal and the margin rule all pass" -ForegroundColor Green
} else {
    Write-Host "timing: $violations violation(s) across $rows clocks and the path list - do not flash" -ForegroundColor Red
}
exit $violations
