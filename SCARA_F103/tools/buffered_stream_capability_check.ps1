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
    $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
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
    if ($hostcap -notmatch "binary_traj=1" -or $hostcap -notmatch "binary_timed=1" -or $hostcap -notmatch "gcode_abt=1") {
        throw "Controller does not advertise binary trajectory, binary timed segments and G1 A/B/T support"
    }

    Write-Host "TX ?"
    $serial.Write("?")
    $status = Read-MatchingLine -Serial $serial -Prefix "<" -WaitMs ([Math]::Max(2000, $TimeoutMs * 4))

    if ($status -notmatch "\|Bf:(\d+),(\d+)\|") {
        throw "Status is missing Bf planner/RX byte capacity"
    }
    $plannerFree = [int]$Matches[1]
    $rxFreeBytes = [int]$Matches[2]
    if ($plannerFree -lt 1 -or $rxFreeBytes -lt 256) {
        throw "Old or insufficient buffer report: planner_free=$plannerFree rx_free_bytes=$rxFreeBytes"
    }
    if ($status -notmatch "\|Q:(\d+)\|") {
        throw "Status is missing planner queue count Q"
    }
    if ($status -notmatch "\|Sq:(\d+),(\d+)\|") {
        throw "Status is missing step segment queue occupancy Sq"
    }
    if ($status -notmatch "\|JU:(\d+),(\d+),(\d+),(\d+)\|") {
        throw "Status is missing four-field underrun diagnostics JU"
    }
    if ($status -notmatch "\|Hz:10000\|") {
        throw "Controller is not reporting the required 10 kHz control tick"
    }

    Write-Host "PASS version: $version"
    Write-Host "PASS host capabilities: binary trajectory, binary timed segments and G1 A/B/T"
    Write-Host "PASS three-layer status: Bf/Q/Sq/JU"
    Write-Host "PASS RX byte capacity: $rxFreeBytes"
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
