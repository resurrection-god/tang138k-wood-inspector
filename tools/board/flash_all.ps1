<#
.SYNOPSIS
    Performs a full build and flashes both FPGA bitstream (0x0) and MCU firmware (0x600000).

.DESCRIPTION
    Order matters: after --run 53 writes the bitstream, the firmware at 0x600000
    must be written again (ERRORS ch. 27.5, 28.6), so the MCU step always follows.

    -ColdBoot finishes with an SRAM load of the same bitstream (--run 2), which
    reconfigures the FPGA and gives a genuine cold boot of what was just flashed.
    Measured 15.09.2026: bitstream to flash ~130 s, firmware ~5 s, SRAM ~30 s.
#>

[CmdletBinding()]
param(
    [switch]$SkipBuild,
    [int]$PlaceOption = -1,
    [switch]$Force,
    [switch]$ColdBoot
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host "=======================================================" -ForegroundColor Magenta
Write-Host " [FULL FLASH] Building & Flashing FPGA + MCU Firmware" -ForegroundColor Magenta
Write-Host "=======================================================" -ForegroundColor Magenta

# 1. FPGA Phase
Write-Host "`n>>> Step 1: FPGA Bitstream" -ForegroundColor Cyan
& "$scriptDir\build_and_flash_fpga.ps1" -SkipBuild:$SkipBuild -PlaceOption $PlaceOption -Force:$Force

# 2. MCU Phase
Write-Host "`n>>> Step 2: MCU Firmware" -ForegroundColor Cyan
& "$scriptDir\build_and_flash_mcu.ps1" -SkipBuild:$SkipBuild

# 3. Optional cold boot
if ($ColdBoot) {
    Write-Host "`n>>> Step 3: Cold boot (SRAM load, --run 2)" -ForegroundColor Cyan
    & "$scriptDir\run_sram.ps1" -SkipBuild
}

Write-Host "`n=======================================================" -ForegroundColor Green
Write-Host " [ALL COMPLETE] FPGA Bitstream and MCU Firmware Flashed!" -ForegroundColor Green
Write-Host "=======================================================" -ForegroundColor Green
