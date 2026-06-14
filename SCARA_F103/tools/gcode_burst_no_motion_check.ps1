param(
    [string]$Port = "AUTO",
    [int]$Baud = 115200,
    [int]$BurstLines = 6,
    [int]$TimeoutMs = 800
)

# Sends only modal G-code. It verifies burst RX/ACK behavior without moving axes.
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

function Get-Checksum {
    param([string]$Line)
    $sum = 0
    foreach ($byte in [System.Text.Encoding]::ASCII.GetBytes($Line)) {
        $sum = ($sum + $byte) -band 0xFF
    }
    return "{0:X2}" -f $sum
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
    Write-Host "Opening $resolvedPort at $Baud 8N1 (no motion) ..."
    $serial.Open()
    Start-Sleep -Milliseconds 300
    $serial.DiscardInBuffer()

    $expected = @{}
    $burst = [System.Text.StringBuilder]::new()
    Write-Host "TX ? (realtime priority probe)"
    [void]$burst.Append("?")
    for ($i = 1; $i -le $BurstLines; $i++) {
        $mode = if (($i % 2) -eq 0) { "G21" } else { "G90" }
        $line = "$mode ;BURST=$i"
        $expected[$line] = Get-Checksum -Line $line
        Write-Host "TX $line"
        [void]$burst.Append($line).Append("`n")
    }
    $serial.Write($burst.ToString())

    $received = @{}
    $pureOkCount = 0
    $firstProtocolResponse = $null
    $deadline = (Get-Date).AddMilliseconds([Math]::Max(3000, $TimeoutMs * ($BurstLines + 2)))
    while ((Get-Date) -lt $deadline -and (($received.Count + $pureOkCount) -lt $BurstLines)) {
        try {
            $rx = $serial.ReadLine().Trim()
            Write-Host "RX $rx"
            if ($null -eq $firstProtocolResponse -and
                ($rx.StartsWith("<") -or $rx.Contains("|Bf:") -or $rx -match "^ok(\s|$)")) {
                $firstProtocolResponse = $rx
            }
            if ($rx.Trim().ToLowerInvariant() -eq "ok") {
                $pureOkCount++
            } elseif ($rx -match "^ok seq=\d+ cs=([0-9A-F]{2}) line=(.*)$") {
                $cs = $Matches[1]
                $line = $Matches[2]
                if (-not $expected.ContainsKey($line)) {
                    throw "Unexpected ACK line: $line"
                }
                if ($expected[$line] -ne $cs) {
                    throw "Checksum mismatch for '$line': expected=$($expected[$line]) actual=$cs"
                }
                $received[$line] = $true
            }
        } catch [TimeoutException] {
        }
    }
    $ackCount = $received.Count + $pureOkCount
    if ($ackCount -ne $BurstLines) {
        throw "Received $ackCount/$BurstLines ACKs"
    }
    if ($null -eq $firstProtocolResponse -or
        (-not $firstProtocolResponse.StartsWith("<") -and -not $firstProtocolResponse.Contains("|Bf:"))) {
        throw "Realtime status did not bypass normal burst ACKs; first response: $firstProtocolResponse"
    }

    $serial.DiscardInBuffer()
    Write-Host "TX STATUS"
    $serial.WriteLine("STATUS")
    $statusDeadline = (Get-Date).AddMilliseconds([Math]::Max(2000, $TimeoutMs * 4))
    $status = $null
    while ((Get-Date) -lt $statusDeadline) {
        try {
            $rx = $serial.ReadLine().Trim()
            Write-Host "RX $rx"
            if ($rx.StartsWith("STAT ")) {
                $status = $rx
                break
            }
        } catch [TimeoutException] {
        }
    }
    if ($null -eq $status) {
        throw "No STATUS response"
    }
    if ($status -notmatch "\brxov=0\b" -or $status -notmatch "\btxd=0\b") {
        throw "Controller reported RX overflow or TX drop"
    }

    Write-Host "PASS realtime status bypassed burst ACKs; burst ACKs=$ackCount rxov=0 txd=0"
    Write-Host "GCODE BURST NO-MOTION CHECK PASS"
    exit 0
} catch {
    Write-Host "GCODE BURST NO-MOTION CHECK FAIL: $($_.Exception.Message)"
    exit 1
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
