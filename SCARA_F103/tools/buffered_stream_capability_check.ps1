param(
    [string]$Port = "AUTO",
    [int]$Baud = 115200,
    [int]$TimeoutMs = 800
)

# Non-motion capability check for the buffered sender/firmware contract.
$ErrorActionPreference = "Stop"

function Resolve-Port {
    param([string]$Requested)
    if ($Requested -and $Requested.ToUpperInvariant() -ne "AUTO") {
        return $Requested
    }
    $ports = @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)
    if ($ports.Count -ne 1) {
        throw "AUTO requires exactly one serial port; found: $($ports -join ', ')"
    }
    return $ports[0]
}

function Read-MatchingLine {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string]$Prefix,
        [int]$WaitMs
    )
    $deadline = (Get-Date).AddMilliseconds($WaitMs)
    while ((Get-Date) -lt $deadline) {
        try {
            $line = $Serial.ReadLine().Trim()
            Write-Host "RX $line"
            if ($line.StartsWith($Prefix)) {
                return $line
            }
        } catch [TimeoutException] {
        }
    }
    throw "Timed out waiting for '$Prefix'"
}

function Send-LineAndRead {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string]$Command,
        [string]$Prefix
    )
    Write-Host "TX $Command"
    $Serial.WriteLine($Command)
    return Read-MatchingLine -Serial $Serial -Prefix $Prefix -WaitMs ([Math]::Max(2000, $TimeoutMs * 4))
}

$resolvedPort = Resolve-Port -Requested $Port
$serial = [System.IO.Ports.SerialPort]::new(
    $resolvedPort,
    $Baud,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.NewLine = "`n"
$serial.ReadTimeout = $TimeoutMs
$serial.WriteTimeout = $TimeoutMs

try {
    Write-Host "Opening $resolvedPort at $Baud 8N1 (non-motion check) ..."
    $serial.Open()
    Start-Sleep -Milliseconds 300
    $serial.DiscardInBuffer()

    $version = Send-LineAndRead -Serial $serial -Command "VERSION" -Prefix "OK VERSION"
    $hostcap = Send-LineAndRead -Serial $serial -Command "HOSTCAP" -Prefix "OK HOSTCAP"
    foreach ($cap in @("grbl_stream=1", "char_count=1", "scara_plan=1", "planner=48", "segments=16", "gcode_arc=1", "jog=1", "dda=1", "hz=10000")) {
        if ($hostcap -notmatch [regex]::Escape($cap)) {
            throw "Controller HOSTCAP is missing $cap"
        }
    }

    Write-Host "TX ?"
    $serial.Write("?")
    $status = Read-MatchingLine -Serial $serial -Prefix "<" -WaitMs ([Math]::Max(2000, $TimeoutMs * 4))

    if ($status -notmatch "\|Bf:(\d+),(\d+)\|") {
        throw "Status is missing Bf planner/RX byte capacity"
    }
    $plannerFree = [int]$Matches[1]
    $rxFreeBytes = [int]$Matches[2]
    if ($plannerFree -lt 1 -or $rxFreeBytes -lt 128) {
        throw "Old or insufficient buffer report: planner_free=$plannerFree rx_free_bytes=$rxFreeBytes"
    }
    foreach ($field in @("MPos:", "JPos:", "FS:", "E:", "H:", "HS:", "A1:", "A2:", "Lz:")) {
        if (-not $status.Contains($field)) {
            throw "Realtime status is missing $field"
        }
    }
    if ($status -notmatch "\|Q:(\d+)\|") {
        throw "Status is missing planner queue count Q"
    }
    if ($status -notmatch "\|Seg:(\d+),(\d+),(\d+),(\d+)\|") {
        throw "Status is missing step segment diagnostics Seg"
    }

    Write-Host "TX STATUS"
    $serial.WriteLine("STATUS")
    $longStatus = Read-MatchingLine -Serial $serial -Prefix "STAT " -WaitMs ([Math]::Max(2000, $TimeoutMs * 4))
    if ($longStatus -notmatch "\bhz=10000\b") {
        throw "Long STATUS is not reporting the required 10 kHz control tick"
    }
    if ($longStatus -notmatch "\bic=(\d+)\b") {
        throw "Long STATUS is missing maximum control ISR cycle telemetry ic"
    }
    $maxTickCycles = [int]$Matches[1]
    if ($maxTickCycles -ge 7200) {
        throw "Control ISR exceeded the 10 kHz cycle budget: max_cycles=$maxTickCycles budget=7200"
    }

    Write-Host "PASS version: $version"
    Write-Host "PASS host capabilities: GRBL stream, SCARA planner, arcs, jog, DDA"
    Write-Host "PASS realtime status: MPos/JPos/Bf/Q/E/Seg/H/HS/A1/A2/Lz"
    Write-Host "PASS RX byte capacity: $rxFreeBytes"
    Write-Host "PASS control ISR cycle budget: $maxTickCycles < 7200"
    Write-Host "BUFFERED STREAM CAPABILITY CHECK PASS"
    exit 0
} catch {
    Write-Host "BUFFERED STREAM CAPABILITY CHECK FAIL: $($_.Exception.Message)"
    exit 1
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
