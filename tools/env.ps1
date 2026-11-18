# Project tool environment for the current PowerShell session (dot-source it):
#   . .\tools\env.ps1
# Changes only this process: nothing is written to the user or machine PATH.
# Details and versions: docs/TOOLCHAIN_SETUP.md.

$AndeSight = "C:\Andestech\AndeSight_RDS_v511"
$Gowin     = "C:\Gowin\Gowin_V1.9.12.03_x64"

# Python first: AndeSight's cygwin\bin holds cygwin symlinks "python"/"python3"
# that Windows cannot start, so with cygwin ahead "python check_layers.py" fails.
$prepend = @(
    "C:\Python314",
    "C:\Python314\Scripts",
    "$AndeSight\cygwin\bin",                                # make, sh; ICEman/gdb need its DLLs (ERRORS 5.11)
    "$AndeSight\toolchains\nds32le-elf-mculib-v5\bin",      # riscv32-elf-gcc/gdb/nm
    "$AndeSight\utils",                                     # nds_ldsag (clean build)
    "$AndeSight\ice"                                        # ICEman
)
$rest = ($env:Path -split ";") | Where-Object { $_ -and ($prepend -notcontains $_) }
$env:Path = (($prepend | Where-Object { Test-Path $_ }) + $rest) -join ";"

# programmer_cli's embedded Python dies on utf-8:surrogateescape (set by some shells).
Remove-Item Env:\PYTHONIOENCODING -ErrorAction SilentlyContinue

$global:GW_SH = "$Gowin\IDE\bin\gw_sh.exe"
$global:PGM   = "$Gowin\Programmer\bin\programmer_cli.exe"
$global:PGM_CABLE = @("--cable-index", "1", "--location", "0")   # onboard FT2232H

$missing = @($GW_SH, $PGM, "$AndeSight\cygwin\bin\make.exe", "$AndeSight\utils\nds_ldsag.exe",
             "$AndeSight\ice\ICEman.exe") | Where-Object { -not (Test-Path $_) }
if ($missing) { Write-Warning ("missing: " + ($missing -join ", ")) }
else { Write-Host "andesprojects env: make, riscv32-elf-*, nds_ldsag, ICEman on PATH; `$GW_SH, `$PGM, `$PGM_CABLE set" }
