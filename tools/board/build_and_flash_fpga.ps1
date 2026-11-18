<#
.SYNOPSIS
    Synthesizes and flashes the Gowin FPGA bitstream to Tang Mega 138K Pro SPI Flash (0x000000).

.DESCRIPTION
    After a build the post-PnR timing gate (fpga\check_timing.ps1) runs and a
    violation stops the flash unless -Force is given.

    A timing PASS is necessary, NOT sufficient (ERRORS ch. 31.36). On 15.09.2026 the
    same netlist gave: place_option 3 - PASS, boots, Ethernet receive degraded
    (56 s stress cycles, RX watchdog 261 hits); place_option 2 - PASS, does not boot
    at all (PC=0, mcause=2); place_option 4 - PASS and healthy. Accept a bitstream
    only after `tools\capture\stp_client.py --stress 10` runs ~9 s per cycle and the
    UART [ETH STATS] "WD cmp" counter stays near zero.

    -PlaceOption sets GOWIN_PLACE_OPTION for build_fpga.tcl (its default is the one
    recorded there as committed).

    Flashing the bitstream to SPI requires reflashing the firmware at 0x600000
    afterwards (ch. 27.5, 28.6) - use flash_all.ps1 for both.
#>

[CmdletBinding()]
param(
    [switch]$SkipBuild,
    [int]$PlaceOption = -1,
    [switch]$Force
)

$ErrorActionPreference = "Stop"

$scriptDir     = Split-Path -Parent $MyInvocation.MyCommand.Path
$workspaceRoot = (Resolve-Path "$scriptDir\..\..").Path
$fpgaDir       = "$workspaceRoot\fpga"
$fpgaBitstream = "$fpgaDir\impl\pnr\scanner.fs"
$gwShell       = "C:\Gowin\Gowin_V1.9.12.04_x64\IDE\bin\gw_sh.exe"
if (-not (Test-Path $gwShell)) {
    $gwShell   = "C:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe"
}
$progCli       = "C:\Gowin\Gowin_V1.9.12.04_x64\Programmer\bin\programmer_cli.exe"
if (-not (Test-Path $progCli)) {
    $progCli   = "C:\Gowin\Gowin_V1.9.12.03_x64\Programmer\bin\programmer_cli.exe"
}

Write-Host "=======================================================" -ForegroundColor Cyan
Write-Host " [FPGA FLASH] Tang Mega 138K Pro Gowin GW5AST-138B" -ForegroundColor Cyan
Write-Host "=======================================================" -ForegroundColor Cyan

if (-not $SkipBuild) {
    Write-Host "[1/3] Synthesizing and routing FPGA design (gw_sh.exe, ~15 min)..." -ForegroundColor Yellow
    if ($PlaceOption -ge 0) { $env:GOWIN_PLACE_OPTION = "$PlaceOption" }
    else { Remove-Item Env:\GOWIN_PLACE_OPTION -ErrorAction SilentlyContinue }
    Push-Location $fpgaDir
    try {
        & $gwShell "build_fpga.tcl"
        if ($LASTEXITCODE -ne 0) {
            Write-Error "FPGA synthesis/PnR failed with exit code $LASTEXITCODE"
        }
    } finally {
        Pop-Location
    }
} else {
    Write-Host "[1/3] Skipping build step (-SkipBuild specified)." -ForegroundColor Gray
}

if (-not (Test-Path $fpgaBitstream)) {
    Write-Error "FPGA bitstream file not found: $fpgaBitstream"
}

Write-Host "[2/3] Post-PnR timing gate (check_timing.ps1)..." -ForegroundColor Yellow
& "$fpgaDir\check_timing.ps1"
$violations = $LASTEXITCODE
if ($violations -ne 0) {
    if ($Force) { Write-Host "      $violations timing violation(s) - flashing anyway (-Force)" -ForegroundColor Red }
    else { Write-Error "$violations timing violation(s): not flashing. Re-run with -Force only on purpose." }
}
Write-Host ("      scanner.fs MD5 " + (Get-FileHash $fpgaBitstream -Algorithm MD5).Hash)

# programmer_cli embeds an old Python that dies on this variable, and a running
# ICEman holds the FT2232H (ch. 31.4).
Remove-Item Env:\PYTHONIOENCODING -ErrorAction SilentlyContinue
Get-Process ICEman, riscv32-elf-gdb -ErrorAction SilentlyContinue | Stop-Process -Force -Confirm:$false

Write-Host "[3/3] Flashing FPGA Bitstream to SPI Flash (addr: 0x000000, freq: 15MHz, ~130 s)..." -ForegroundColor Yellow
& $progCli --device "GW5AST-138B" --freq 15MHz --run 53 --fsFile $fpgaBitstream --spiaddr 0x000000

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n[SUCCESS] FPGA Bitstream successfully flashed to SPI Flash (0x000000)!" -ForegroundColor Green
    Write-Host "          Reflash the firmware at 0x600000 next, then accept the build by network stress (see .DESCRIPTION)." -ForegroundColor Yellow
} else {
    Write-Error "Programmer CLI failed with exit code $LASTEXITCODE"
}
