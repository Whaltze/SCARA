param(
    [string]$Port = "AUTO",
    [ValidateSet("stlink", "cmsis-dap")]
    [string]$Probe = "stlink",
    [switch]$ConfirmMotorPowerOff
)

# Flashing resets the MCU and this firmware enables ENA on boot. Require an
# explicit declaration that driver/motor power is off before programming.
$ErrorActionPreference = "Stop"

if (-not $ConfirmMotorPowerOff) {
    Write-Host "REFUSED: disconnect motor-driver power and rerun with -ConfirmMotorPowerOff."
    exit 2
}

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$elf = Join-Path $projectRoot "build\Debug\SCARA_F103.elf"
if (-not (Test-Path -LiteralPath $elf)) {
    throw "Firmware ELF does not exist: $elf"
}

$interface = if ($Probe -eq "stlink") { "interface/stlink.cfg" } else { "interface/cmsis-dap.cfg" }

Push-Location $projectRoot
try {
    Write-Host "Flashing v0.26.0 with $Probe; motor-driver power confirmed off."
    & openocd -f $interface -f target/stm32f1x.cfg -c "program build/Debug/SCARA_F103.elf verify reset exit"
    if ($LASTEXITCODE -ne 0) {
        throw "OpenOCD flash/verify failed exit=$LASTEXITCODE"
    }

    Start-Sleep -Seconds 2
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "buffered_stream_capability_check.ps1") -Port $Port
    if ($LASTEXITCODE -ne 0) {
        throw "Buffered stream capability check failed"
    }
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "gcode_burst_no_motion_check.ps1") -Port $Port -BurstLines 6
    if ($LASTEXITCODE -ne 0) {
        throw "No-motion G-code burst check failed"
    }

    Write-Host "FLASH AND BUFFERED VERIFY PASS"
    Write-Host "Keep driver power off until the mechanism is clear for motion testing."
    exit 0
} catch {
    Write-Host "FLASH AND BUFFERED VERIFY FAIL: $($_.Exception.Message)"
    exit 1
} finally {
    Pop-Location
}
