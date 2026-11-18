<#
.SYNOPSIS
    Loads the FPGA bitstream directly into volatile SRAM (Run Mode 2) for rapid testing without flash wear.

.DESCRIPTION
    --run 2 also reconfigures the FPGA, which re-asserts the power-on reset
    sequencer: it is the reliable cold boot of the whole board (the firmware is
    reloaded from SPI 0x600000). gdb_debugger.py --reset-halt is NOT a restart.

    After a build the timing gate runs and a violation stops the load unless
    -Force is given. A timing PASS is still not proof the build is usable
    (ERRORS ch. 31.36) - check boot and network stress.
#>

[CmdletBinding()]
param(
    [switch]$SkipBuild,
    [int]$PlaceOption = -1,
    [string]$UsbLocation = "4641",
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

Write-Host "=======================================================" -ForegroundColor Yellow
Write-Host " [SRAM RUN] Direct Volatile FPGA Programming (Mode 2)" -ForegroundColor Yellow
Write-Host "=======================================================" -ForegroundColor Yellow

if (-not $SkipBuild) {
    Write-Host "[1/2] Synthesizing FPGA design..." -ForegroundColor Yellow
    if ($PlaceOption -ge 0) { $env:GOWIN_PLACE_OPTION = "$PlaceOption" }
    else { Remove-Item Env:\GOWIN_PLACE_OPTION -ErrorAction SilentlyContinue }
    Push-Location $fpgaDir
    try {
        & $gwShell "build_fpga.tcl"
        if ($LASTEXITCODE -ne 0) {
            Write-Error "FPGA synthesis failed with exit code $LASTEXITCODE"
        }
    } finally {
        Pop-Location
    }
    & "$fpgaDir\check_timing.ps1"
    if ($LASTEXITCODE -ne 0 -and -not $Force) {
        Write-Error "$LASTEXITCODE timing violation(s): not loading. Re-run with -Force only on purpose."
    }
}

if (-not (Test-Path $fpgaBitstream)) {
    Write-Error "Bitstream file not found: $fpgaBitstream"
}

# programmer_cli embeds an old Python that dies on this variable, and a running
# ICEman holds the FT2232H (ch. 31.4).
Remove-Item Env:\PYTHONIOENCODING -ErrorAction SilentlyContinue
Get-Process ICEman, riscv32-elf-gdb -ErrorAction SilentlyContinue | Stop-Process -Force -Confirm:$false

Write-Host "[2/2] Verifying target JTAG cable (USB location $UsbLocation) and loading SRAM..." -ForegroundColor Yellow
$scanOut = & $progCli --cable-index 1 --location $UsbLocation --scan 2>&1 | Out-String
if ($LASTEXITCODE -ne 0 -or $scanOut -notmatch "0x0001081B") {
    Write-Error "Target board at USB location $UsbLocation (GW5AST-138B) not found! Refusing to flash another board."
}
& $progCli --device "GW5AST-138B" --cable-index 1 --location $UsbLocation --run 2 --fsFile $fpgaBitstream

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n[SUCCESS] Bitstream successfully loaded into SRAM! Board boots; network is up ~10-15 s later." -ForegroundColor Green
} else {
    Write-Error "Programmer CLI failed with exit code $LASTEXITCODE"
}
