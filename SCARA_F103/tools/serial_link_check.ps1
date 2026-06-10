param(
    [string]$Port = "",
    [int]$Baud = 115200,
    [int]$TimeoutMs = 800,
    [int]$Repeat = 5,
    [switch]$ListPorts
)

# 最基础串口链路检查：用于确认端口、波特率、VERSION/PING/STATUS 是否可用。

$ErrorActionPreference = "Stop"

function Show-Ports {
    $ports = @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)
    if ($ports.Count -eq 0) {
        Write-Host "No serial ports found."
    } else {
        Write-Host "Serial ports:"
        foreach ($p in $ports) {
            Write-Host "  $p"
        }
    }
}

function Read-Available {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [int]$DurationMs
    )

    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    $text = ""
    while ($timer.ElapsedMilliseconds -lt $DurationMs) {
        if ($Serial.BytesToRead -gt 0) {
            $text += $Serial.ReadExisting()
        } else {
            Start-Sleep -Milliseconds 20
        }
    }
    $timer.Stop()
    return $text.Trim()
}

function Read-Line-Safe {
    param([System.IO.Ports.SerialPort]$Serial)

    try {
        return $Serial.ReadLine().Trim()
    } catch [TimeoutException] {
        return "<timeout>"
    }
}

function Send-Command {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string]$Command,
        [string[]]$AcceptedPrefixes
    )

    $Serial.DiscardInBuffer()
    $Serial.WriteLine($Command)
    $deadline = (Get-Date).AddMilliseconds([Math]::Max(1000, $TimeoutMs * 4))
    $response = "<timeout>"
    $ok = $false
    while ((Get-Date) -lt $deadline) {
        $response = Read-Line-Safe -Serial $Serial
        if ($response.StartsWith("<")) {
            continue
        }
        foreach ($prefix in $AcceptedPrefixes) {
            if ($response.StartsWith($prefix)) {
                $ok = $true
                break
            }
        }
        if ($ok -or $response -eq "<timeout>") {
            break
        }
    }

    $status = if ($ok) { "PASS" } else { "FAIL" }
    Write-Host ("{0} TX '{1}' RX '{2}'" -f $status, $Command, $response)
    return @{
        Ok = $ok
        Response = $response
    }
}

if ($ListPorts) {
    Show-Ports
    if ($Port.Length -eq 0) {
        exit 0
    }
}

if ($Port.Length -eq 0) {
    Show-Ports
    Write-Host ""
    Write-Host "Usage:"
    Write-Host "  powershell -NoProfile -ExecutionPolicy Bypass -File tools\serial_link_check.ps1 -Port COM5"
    Write-Host "  powershell -NoProfile -ExecutionPolicy Bypass -File tools\serial_link_check.ps1 -ListPorts"
    exit 2
}

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = $TimeoutMs
$serial.WriteTimeout = $TimeoutMs

$failed = 0
try {
    Write-Host "Opening $Port at $Baud 8N1 ..."
    $serial.Open()
    Write-Host "PASS port opened"

    Write-Host "INFO waiting briefly for boot banner or existing output ..."
    Start-Sleep -Milliseconds 1200
    $banner = Read-Available -Serial $serial -DurationMs 300
    if ($banner.Length -gt 0) {
        Write-Host "BOOT $banner"
    } else {
        Write-Host "INFO no boot banner captured; board may already be running"
    }

    Write-Host "INFO sending VERSION ..."
    $version = Send-Command -Serial $serial -Command "VERSION" -AcceptedPrefixes @("OK VERSION")
    if (-not $version.Ok) { $failed++ }

    for ($i = 1; $i -le $Repeat; $i++) {
        Write-Host "INFO sending PING $i/$Repeat ..."
        $ping = Send-Command -Serial $serial -Command "PING" -AcceptedPrefixes @("OK PONG")
        if (-not $ping.Ok) { $failed++ }
        Start-Sleep -Milliseconds 50
    }

    Write-Host "INFO sending STATUS ..."
    $status = Send-Command -Serial $serial -Command "STATUS" -AcceptedPrefixes @("STAT")
    if (-not $status.Ok) {
        $failed++
    } else {
        if ($status.Response -match "rx_ov=([0-9]+)") {
            Write-Host "INFO rx_ov=$($Matches[1])"
        }
        if ($status.Response -match "tx_drop=([0-9]+)") {
            Write-Host "INFO tx_drop=$($Matches[1])"
        }
        if ($status.Response -match "tx_q=([0-9]+)") {
            Write-Host "INFO tx_q=$($Matches[1])"
        }
    }
} catch {
    Write-Host "FAIL $($_.Exception.Message)"
    $failed++
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
        Write-Host "INFO port closed"
    }
}

if ($failed -eq 0) {
    Write-Host "SERIAL LINK PASS"
    exit 0
}

Write-Host "SERIAL LINK FAIL failures=$failed"
exit 1
