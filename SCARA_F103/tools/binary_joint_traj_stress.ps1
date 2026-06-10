param(
    [string]$Port = "COM13",
    [int]$Baud = 115200,
    [int]$TimeoutMs = 1500,
    [int]$Count = 3000,
    [int]$ChunkPoints = 20,
    [int]$FeedPps = 600,
    [int]$StartP1 = 0,
    [int]$StartP2 = 0,
    [double]$MaxErrorMm = 1.0,
    [string]$CsvPath = "",
    [switch]$ZeroBeforeRun,
    [switch]$EnableMotion,
    [switch]$KeepEnabled,
    [switch]$HostTimed,
    [int]$TimedTicks = 100
)

$ErrorActionPreference = "Stop"

$SOF0 = 0xA5
$SOF1 = 0x5A
$VER = 1
$TYPE_HELLO = 0x01
$TYPE_BEGIN = 0x10
$TYPE_CHUNK = 0x11
$TYPE_VALIDATE = 0x12
$TYPE_RUN = 0x13
$TYPE_ABORT = 0x14
$TYPE_STATUS = 0x15
$TYPE_ACK = 0x80
$TYPE_NACK = 0x81
$TYPE_STATUS_RSP = 0x82
$PPR1 = 3200.0
$PPR2 = 3200.0
$ZERO1_MRAD = 2251.0
$ZERO2_MRAD = 890.0
$BASE_MM = 150.0
$ACTIVE_MM = 160.0
$PASSIVE_MM = 200.0
$expected = New-Object System.Collections.Generic.List[object]
$samples = New-Object System.Collections.Generic.List[object]

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

function Update-Crc16 {
    param([int]$Crc, [int]$Byte)
    $crc = $Crc -bxor ($Byte -band 0xFF)
    for ($i = 0; $i -lt 8; $i++) {
        if (($crc -band 1) -ne 0) {
            $crc = (($crc -shr 1) -bxor 0xA001) -band 0xFFFF
        } else {
            $crc = ($crc -shr 1) -band 0xFFFF
        }
    }
    return $crc
}

function Get-Crc16 {
    param([byte[]]$Data, [int]$Offset, [int]$Length)
    $crc = 0xFFFF
    for ($i = 0; $i -lt $Length; $i++) {
        $crc = Update-Crc16 -Crc $crc -Byte $Data[$Offset + $i]
    }
    return $crc
}

function Write-U16 {
    param([System.Collections.Generic.List[byte]]$Bytes, [int]$Value)
    $Bytes.Add([byte]($Value -band 0xFF))
    $Bytes.Add([byte](($Value -shr 8) -band 0xFF))
}

function Write-U32 {
    param([System.Collections.Generic.List[byte]]$Bytes, [long]$Value)
    $Bytes.Add([byte]($Value -band 0xFF))
    $Bytes.Add([byte](($Value -shr 8) -band 0xFF))
    $Bytes.Add([byte](($Value -shr 16) -band 0xFF))
    $Bytes.Add([byte](($Value -shr 24) -band 0xFF))
}

function Write-I32 {
    param([System.Collections.Generic.List[byte]]$Bytes, [int]$Value)
    foreach ($b in [System.BitConverter]::GetBytes([int]$Value)) {
        $Bytes.Add($b)
    }
}

function Get-TestPulsePoint {
    param([int]$Index, [int]$Total)
    $u = [double]$Index / [Math]::Max(1, $Total - 1)
    $p1 = [int][Math]::Round(800.0 * [Math]::Sin(2.0 * [Math]::PI * $u))
    $p2 = [int][Math]::Round(800.0 * [Math]::Cos(2.0 * [Math]::PI * $u))
    return [pscustomobject]@{ P1 = $p1; P2 = $p2 }
}

function Convert-PulseToXY {
    param([int]$P1, [int]$P2)
    $theta1Mrad = [int]([Math]::Truncate(($P1 * 6283.0) / $PPR1)) + [int]$ZERO1_MRAD
    $theta2Mrad = [int]([Math]::Truncate(($P2 * 6283.0) / $PPR2)) + [int]$ZERO2_MRAD
    $t1 = $theta1Mrad / 1000.0
    $t2 = $theta2Mrad / 1000.0
    $halfBase = $BASE_MM * 0.5
    $ex1 = -$halfBase + $ACTIVE_MM * [Math]::Cos($t1)
    $ey1 = $ACTIVE_MM * [Math]::Sin($t1)
    $ex2 = $halfBase + $ACTIVE_MM * [Math]::Cos($t2)
    $ey2 = $ACTIVE_MM * [Math]::Sin($t2)
    $dx = $ex2 - $ex1
    $dy = $ey2 - $ey1
    $d = [Math]::Sqrt($dx * $dx + $dy * $dy)
    if ($d -le 1e-9) { return $null }
    $a = ($PASSIVE_MM * $PASSIVE_MM - $PASSIVE_MM * $PASSIVE_MM + $d * $d) / (2.0 * $d)
    $h2 = $PASSIVE_MM * $PASSIVE_MM - $a * $a
    if ($h2 -lt 0.0) { return $null }
    $h = [Math]::Sqrt($h2)
    $mx = $ex1 + $a * $dx / $d
    $my = $ey1 + $a * $dy / $d
    $rx = -$dy / $d
    $ry = $dx / $d
    $ix1 = $mx + $h * $rx
    $iy1 = $my + $h * $ry
    $ix2 = $mx - $h * $rx
    $iy2 = $my - $h * $ry
    if ($iy1 -ge $iy2) {
        return [pscustomobject]@{ X = $ix1; Y = $iy1 }
    }
    return [pscustomobject]@{ X = $ix2; Y = $iy2 }
}

function Add-ExpectedPulsePoint {
    param([int]$P1, [int]$P2)
    $xy = Convert-PulseToXY -P1 $P1 -P2 $P2
    if ($null -ne $xy) {
        $script:expected.Add($xy)
    }
}

function Add-ExpectedPulseSegment {
    param([int]$FromP1, [int]$FromP2, [int]$ToP1, [int]$ToP2)
    $d1 = $ToP1 - $FromP1
    $d2 = $ToP2 - $FromP2
    $steps = [Math]::Max([Math]::Abs($d1), [Math]::Abs($d2))
    if ($steps -le 0) { return }
    for ($i = 1; $i -le $steps; $i++) {
        $p1 = [int][Math]::Round($FromP1 + ($d1 * [double]$i / [double]$steps))
        $p2 = [int][Math]::Round($FromP2 + ($d2 * [double]$i / [double]$steps))
        Add-ExpectedPulsePoint -P1 $p1 -P2 $p2
    }
}

function Build-ExpectedTrajectory {
    $script:expected.Clear()
    $prevP1 = $StartP1
    $prevP2 = $StartP2
    Add-ExpectedPulsePoint -P1 $prevP1 -P2 $prevP2
    for ($i = 0; $i -lt $Count; $i++) {
        $point = Get-TestPulsePoint -Index $i -Total $Count
        Add-ExpectedPulseSegment -FromP1 $prevP1 -FromP2 $prevP2 -ToP1 $point.P1 -ToP2 $point.P2
        $prevP1 = $point.P1
        $prevP2 = $point.P2
    }
}

function Project-To-Segment {
    param([double]$Px, [double]$Py, [double]$Ax, [double]$Ay, [double]$Bx, [double]$By)
    $vx = $Bx - $Ax
    $vy = $By - $Ay
    $den = $vx * $vx + $vy * $vy
    if ($den -le 1e-12) { return [pscustomobject]@{ X = $Ax; Y = $Ay } }
    $t = (($Px - $Ax) * $vx + ($Py - $Ay) * $vy) / $den
    if ($t -lt 0.0) { $t = 0.0 }
    if ($t -gt 1.0) { $t = 1.0 }
    return [pscustomobject]@{ X = $Ax + $vx * $t; Y = $Ay + $vy * $t }
}

function Add-FeedbackSample {
    param([double]$X, [double]$Y)
    if ($script:expected.Count -lt 2) { return }
    $best = $null
    for ($i = 0; $i -lt ($script:expected.Count - 1); $i++) {
        $a = $script:expected[$i]
        $b = $script:expected[$i + 1]
        $p = Project-To-Segment -Px $X -Py $Y -Ax $a.X -Ay $a.Y -Bx $b.X -By $b.Y
        $dx = $X - $p.X
        $dy = $Y - $p.Y
        $e2 = $dx * $dx + $dy * $dy
        if ($null -eq $best -or $e2 -lt $best.E2) {
            $best = [pscustomobject]@{ X = $p.X; Y = $p.Y; Dx = $dx; Dy = $dy; E2 = $e2 }
        }
    }
    if ($null -ne $best) {
        $script:samples.Add([pscustomobject]@{
            Index = $script:samples.Count + 1
            X = $X
            Y = $Y
            ExpectedX = $best.X
            ExpectedY = $best.Y
            Dx = $best.Dx
            Dy = $best.Dy
            Error = [Math]::Sqrt($best.E2)
        })
    }
}

function Export-FeedbackCsv {
    if ([string]::IsNullOrWhiteSpace($CsvPath)) { return }
    $dir = Split-Path -Parent $CsvPath
    if (-not [string]::IsNullOrWhiteSpace($dir) -and -not (Test-Path -LiteralPath $dir)) {
        New-Item -ItemType Directory -Path $dir | Out-Null
    }
    $script:samples | Export-Csv -LiteralPath $CsvPath -NoTypeInformation -Encoding UTF8
    Write-Host "CSV $CsvPath"
}

function Parse-AsciiStatus {
    param([string]$Line)
    if ($Line -match "M:([-0-9.]+),([-0-9.]+)") {
        Add-FeedbackSample -X ([double]$Matches[1]) -Y ([double]$Matches[2])
    }
}

function Poll-AsciiStatus {
    param([System.IO.Ports.SerialPort]$Serial)
    $oldReadTimeout = $Serial.ReadTimeout
    $Serial.ReadTimeout = 20
    $Serial.Write("?")
    try {
        $deadline = (Get-Date).AddMilliseconds(20)
        while ((Get-Date) -lt $deadline) {
            try {
                $line = $Serial.ReadLine().Trim()
            } catch [TimeoutException] {
                return
            }
            if ($line.StartsWith("<")) {
                Parse-AsciiStatus -Line $line
                return
            }
        }
    } finally {
        $Serial.ReadTimeout = $oldReadTimeout
    }
}

function Get-ErrorSummary {
    if ($script:samples.Count -eq 0) {
        return [pscustomobject]@{ Count = 0; MaxX = 0.0; MaxY = 0.0; MaxE = 0.0; RmsX = 0.0; RmsY = 0.0; RmsE = 0.0 }
    }
    $sumX2 = 0.0
    $sumY2 = 0.0
    $sumE2 = 0.0
    $maxX = 0.0
    $maxY = 0.0
    $maxE = 0.0
    foreach ($s in $script:samples) {
        $maxX = [Math]::Max($maxX, [Math]::Abs($s.Dx))
        $maxY = [Math]::Max($maxY, [Math]::Abs($s.Dy))
        $maxE = [Math]::Max($maxE, $s.Error)
        $sumX2 += $s.Dx * $s.Dx
        $sumY2 += $s.Dy * $s.Dy
        $sumE2 += $s.Error * $s.Error
    }
    return [pscustomobject]@{
        Count = $script:samples.Count
        MaxX = $maxX
        MaxY = $maxY
        MaxE = $maxE
        RmsX = [Math]::Sqrt($sumX2 / [Math]::Max(1, $script:samples.Count))
        RmsY = [Math]::Sqrt($sumY2 / [Math]::Max(1, $script:samples.Count))
        RmsE = [Math]::Sqrt($sumE2 / [Math]::Max(1, $script:samples.Count))
    }
}

function New-Frame {
    param([int]$Type, [int]$Seq, [byte[]]$Payload = @())
    $bytes = [System.Collections.Generic.List[byte]]::new()
    $bytes.Add([byte]$SOF0)
    $bytes.Add([byte]$SOF1)
    $bytes.Add([byte]$VER)
    $bytes.Add([byte]$Type)
    Write-U16 -Bytes $bytes -Value $Seq
    Write-U16 -Bytes $bytes -Value $Payload.Length
    foreach ($b in $Payload) { $bytes.Add($b) }
    $arr = $bytes.ToArray()
    $crc = Get-Crc16 -Data $arr -Offset 2 -Length ($arr.Length - 2)
    Write-U16 -Bytes $bytes -Value $crc
    return $bytes.ToArray()
}

function Read-U16 {
    param([byte[]]$Data, [int]$Offset)
    $b0 = [int]$Data[$Offset]
    $b1 = [int]$Data[$Offset + 1]
    return [int]($b0 -bor ($b1 -shl 8))
}

function Read-U32 {
    param([byte[]]$Data, [int]$Offset)
    $b0 = [uint32]$Data[$Offset]
    $b1 = [uint32]$Data[$Offset + 1]
    $b2 = [uint32]$Data[$Offset + 2]
    $b3 = [uint32]$Data[$Offset + 3]
    return [uint32]($b0 -bor ($b1 -shl 8) -bor ($b2 -shl 16) -bor ($b3 -shl 24))
}

function Read-BinaryFrame {
    param([System.IO.Ports.SerialPort]$Serial, [int]$DeadlineMs = 3000)
    $deadline = (Get-Date).AddMilliseconds($DeadlineMs)
    while ((Get-Date) -lt $deadline) {
        try {
            $b = $Serial.ReadByte()
        } catch [TimeoutException] {
            continue
        }
        if ($b -ne $SOF0) { continue }
        try { $b2 = $Serial.ReadByte() } catch [TimeoutException] { continue }
        if ($b2 -ne $SOF1) { continue }

        $header = New-Object byte[] 6
        for ($i = 0; $i -lt 6; $i++) { $header[$i] = [byte]$Serial.ReadByte() }
        $len = Read-U16 -Data $header -Offset 4
        $payload = New-Object byte[] $len
        for ($i = 0; $i -lt $len; $i++) { $payload[$i] = [byte]$Serial.ReadByte() }
        $crcBytes = New-Object byte[] 2
        $crcBytes[0] = [byte]$Serial.ReadByte()
        $crcBytes[1] = [byte]$Serial.ReadByte()
        $rxCrc = Read-U16 -Data $crcBytes -Offset 0

        $check = [System.Collections.Generic.List[byte]]::new()
        foreach ($x in $header) { $check.Add($x) }
        foreach ($x in $payload) { $check.Add($x) }
        $calc = Get-Crc16 -Data $check.ToArray() -Offset 0 -Length $check.Count
        if ($rxCrc -ne $calc) { throw "RX CRC mismatch rx=$rxCrc calc=$calc" }

        return @{
            Ver = $header[0]
            Type = $header[1]
            Seq = Read-U16 -Data $header -Offset 2
            Payload = $payload
        }
    }
    throw "Timeout waiting binary frame"
}

function Send-Frame {
    param([System.IO.Ports.SerialPort]$Serial, [int]$Type, [int]$Seq, [byte[]]$Payload = @())
    $frame = New-Frame -Type $Type -Seq $Seq -Payload $Payload
    $Serial.Write($frame, 0, $frame.Length)
    $rx = Read-BinaryFrame -Serial $Serial -DeadlineMs 5000
    if ($rx.Type -ne $TYPE_ACK -and $rx.Type -ne $TYPE_STATUS_RSP) {
        if ($rx.Type -eq $TYPE_NACK) {
            $err = if ($rx.Payload.Length -ge 2) { $rx.Payload[1] } else { -1 }
            throw ("NACK type=0x{0:X2} seq={1} err={2}" -f $Type, $Seq, $err)
        }
        throw ("Unexpected frame type=0x{0:X2}" -f $rx.Type)
    }
    if ($rx.Payload.Length -ge 2 -and $rx.Payload[1] -ne 0) {
        throw ("ACK error type=0x{0:X2} seq={1} err={2}" -f $Type, $Seq, $rx.Payload[1])
    }
    return $rx
}

function New-BeginPayload {
    param([int]$Total)
    $bytes = [System.Collections.Generic.List[byte]]::new()
    Write-U32 -Bytes $bytes -Value $Total
    return $bytes.ToArray()
}

function New-PointPayload {
    param([int]$Start, [int]$Take, [int]$Total, [int]$Pps)
    $bytes = [System.Collections.Generic.List[byte]]::new()
    for ($i = 0; $i -lt $Take; $i++) {
        $index = $Start + $i
        $u = [double]$index / [Math]::Max(1, $Total - 1)
        $point = Get-TestPulsePoint -Index $index -Total $Total
        $p1 = $point.P1
        $p2 = $point.P2
        $value = $Pps
        $flags = 0
        if ($HostTimed) {
            if ($index -eq 0) {
                $prevP1 = $StartP1
                $prevP2 = $StartP2
            } else {
                $previous = Get-TestPulsePoint -Index ($index - 1) -Total $Total
                $prevP1 = $previous.P1
                $prevP2 = $previous.P2
            }
            $events = [Math]::Max([Math]::Abs($p1 - $prevP1), [Math]::Abs($p2 - $prevP2))
            $value = [Math]::Max($TimedTicks, $events)
            $flags = 0x0004
        }
        Write-I32 -Bytes $bytes -Value $p1
        Write-I32 -Bytes $bytes -Value $p2
        Write-U16 -Bytes $bytes -Value $value
        Write-U16 -Bytes $bytes -Value $flags
    }
    return $bytes.ToArray()
}

function Send-Ascii {
    param([System.IO.Ports.SerialPort]$Serial, [string]$Line, [string[]]$Prefixes)
    $Serial.WriteLine($Line)
    $deadline = (Get-Date).AddMilliseconds(3000)
    while ((Get-Date) -lt $deadline) {
        try { $rx = $Serial.ReadLine().Trim() } catch [TimeoutException] { continue }
        foreach ($prefix in $Prefixes) {
            if ($rx.StartsWith($prefix)) {
                Write-Host "RX $rx"
                return $rx
            }
        }
    }
    throw "Timeout waiting ASCII response for $Line"
}

$Port = Resolve-Port -Requested $Port
$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = $TimeoutMs
$serial.WriteTimeout = $TimeoutMs
$seq = 1
$sent = 0

try {
    Write-Host "Opening $Port at $Baud 8N1 ..."
    Write-Host ("Binary joint trajectory stress: points={0} chunk={1} mode={2} value={3} max_error={4:F3}mm csv={5}" -f $Count, $ChunkPoints, $(if ($HostTimed) { "host_timed" } else { "velocity" }), $(if ($HostTimed) { $TimedTicks } else { $FeedPps }), $MaxErrorMm, $(if ($CsvPath) { $CsvPath } else { "-" }))
    $serial.Open()
    Start-Sleep -Milliseconds 300
    $serial.DiscardInBuffer()

    Send-Ascii -Serial $serial -Line "VERSION" -Prefixes @("OK VERSION") | Out-Null
    $hostcap = Send-Ascii -Serial $serial -Line "HOSTCAP" -Prefixes @("OK HOSTCAP")
    if ($hostcap -notmatch "binary_traj=1" -or $hostcap -notmatch "(?:control_hz|hz)=10000") {
        throw "Controller does not report binary_traj=1 and a 10 kHz control tick"
    }
    if ($hostcap -notmatch "ppr1=(\d+) ppr2=(\d+)") {
        throw "Controller does not report both axis PPR values"
    }
    $PPR1 = [double]$Matches[1]
    $PPR2 = [double]$Matches[2]
    Build-ExpectedTrajectory
    Write-Host ("Using controller PPR: ppr1={0} ppr2={1}" -f $PPR1, $PPR2)
    Send-Ascii -Serial $serial -Line "CLEAR_ERROR" -Prefixes @("OK CLEAR_ERROR") | Out-Null
    if ($ZeroBeforeRun) {
        Send-Ascii -Serial $serial -Line "ZERO" -Prefixes @("OK ZERO") | Out-Null
    }
    if ($EnableMotion) {
        Send-Ascii -Serial $serial -Line "ENABLE 1" -Prefixes @("OK ENABLE 1") | Out-Null
    }

    Start-Sleep -Milliseconds 100
    $serial.DiscardInBuffer()

    Send-Frame -Serial $serial -Type $TYPE_HELLO -Seq $seq | Out-Null; $seq++
    Send-Frame -Serial $serial -Type $TYPE_BEGIN -Seq $seq -Payload (New-BeginPayload -Total $Count) | Out-Null; $seq++

    $prefill = [Math]::Min($Count, 100)
    while ($sent -lt $prefill) {
        $take = [Math]::Min($ChunkPoints, $prefill - $sent)
        Send-Frame -Serial $serial -Type $TYPE_CHUNK -Seq $seq -Payload (New-PointPayload -Start $sent -Take $take -Total $Count -Pps $FeedPps) | Out-Null
        $seq++
        $sent += $take
    }
    Send-Frame -Serial $serial -Type $TYPE_VALIDATE -Seq $seq | Out-Null; $seq++

    if (-not $EnableMotion) {
        Write-Host "BINARY_TRAJ_LINK PASS uploaded=$sent/$Count motion=disabled"
        exit 0
    }

    Send-Frame -Serial $serial -Type $TYPE_RUN -Seq $seq | Out-Null; $seq++
    $started = Get-Date
    $underrunTicks = 0
    $maxDispatchGap = 0
    $minBuffer = 0
    $lastProgressExecuted = -1
    $loopCount = 0
    while ($true) {
        $loopCount++
        $status = Send-Frame -Serial $serial -Type $TYPE_STATUS -Seq $seq
        $seq++
        $payload = $status.Payload
        $queued = Read-U16 -Data $payload -Offset 2
        $free = Read-U16 -Data $payload -Offset 4
        $accepted = [int](Read-U32 -Data $payload -Offset 6)
        $executed = [int](Read-U32 -Data $payload -Offset 10)
        $state = $payload[18]
        $underrunTicks = 0
        $maxDispatchGap = 0
        $minBuffer = 0
        if ($payload.Length -ge 30) {
            $underrunTicks = [int](Read-U32 -Data $payload -Offset 20)
            $maxDispatchGap = [int](Read-U32 -Data $payload -Offset 24)
            $minBuffer = [int](Read-U16 -Data $payload -Offset 28)
        }

        while ($sent -lt $Count -and $free -ge $ChunkPoints) {
            $take = [Math]::Min($ChunkPoints, $Count - $sent)
            Send-Frame -Serial $serial -Type $TYPE_CHUNK -Seq $seq -Payload (New-PointPayload -Start $sent -Take $take -Total $Count -Pps $FeedPps) | Out-Null
            $seq++
            $sent += $take
            $free -= $take
        }

        if (($loopCount % 20) -eq 0) {
            Poll-AsciiStatus -Serial $serial
        }
        $summary = Get-ErrorSummary
        $elapsed = ((Get-Date) - $started).TotalSeconds
        if ($executed -ge ($lastProgressExecuted + 100) -or $executed -ge $Count) {
            $lastProgressExecuted = $executed
            Write-Host ("PROGRESS sent={0}/{1} accepted={2} executed={3} queued={4} underrun={5} max_gap={6} min_buf={7} samples={8} max={9:F4}mm rms={10:F4}mm elapsed={11:F1}s" -f $sent, $Count, $accepted, $executed, $queued, $underrunTicks, $maxDispatchGap, $minBuffer, $summary.Count, $summary.MaxE, $summary.RmsE, $elapsed)
        }
        if ($executed -ge $Count -and $state -eq 4) { break }
        Start-Sleep -Milliseconds 5
    }

    if (-not $KeepEnabled) {
        Send-Ascii -Serial $serial -Line "ENABLE 0" -Prefixes @("OK ENABLE 0") | Out-Null
    }
    $finalStatus = Send-Ascii -Serial $serial -Line "STATUS" -Prefixes @("STAT ")
    if ($finalStatus -notmatch "ic=(\d+)") {
        throw "Final status is missing ISR cycle telemetry"
    }
    $maxTickCycles = [int]$Matches[1]
    if ($finalStatus -notmatch "p=(-?\d+),(-?\d+)") {
        throw "Final status is missing pulse position"
    }
    $finalP1 = [int]$Matches[1]
    $finalP2 = [int]$Matches[2]
    $expectedFinal = Get-TestPulsePoint -Index ($Count - 1) -Total $Count
    $finalSummary = Get-ErrorSummary
    Write-Host ("BINARY_FEEDBACK_ERROR samples={0} max_x={1:F4} max_y={2:F4} rms_x={3:F4} rms_y={4:F4} max_norm={5:F4} rms_norm={6:F4}" -f $finalSummary.Count, $finalSummary.MaxX, $finalSummary.MaxY, $finalSummary.RmsX, $finalSummary.RmsY, $finalSummary.MaxE, $finalSummary.RmsE)
    Write-Host ("BINARY_JOINT_DIAG underrun_ticks={0} max_dispatch_gap_ticks={1} min_buffer={2}" -f $underrunTicks, $maxDispatchGap, $minBuffer)
    Export-FeedbackCsv
    if ($finalSummary.Count -eq 0) {
        throw "No feedback samples were captured"
    }
    if ($finalSummary.MaxE -gt $MaxErrorMm) {
        throw ("Binary feedback error exceeded limit: max={0:F4}mm limit={1:F4}mm" -f $finalSummary.MaxE, $MaxErrorMm)
    }
    if ($underrunTicks -ne 0 -or $minBuffer -lt 1) {
        throw "Binary stream underrun detected: ticks=$underrunTicks min_buffer=$minBuffer"
    }
    if ($maxDispatchGap -gt 1) {
        throw "Binary dispatch gap exceeded one control tick: max_gap=$maxDispatchGap"
    }
    if ($maxTickCycles -ge 7200) {
        throw "Control ISR exceeded 10 kHz budget during motion: max_cycles=$maxTickCycles budget=7200"
    }
    if ($finalP1 -ne $expectedFinal.P1 -or $finalP2 -ne $expectedFinal.P2) {
        throw "Final pulse mismatch: actual=$finalP1,$finalP2 expected=$($expectedFinal.P1),$($expectedFinal.P2)"
    }
    Write-Host "BINARY_ENDPOINT actual=$finalP1,$finalP2 expected=$($expectedFinal.P1),$($expectedFinal.P2)"
    Write-Host "BINARY_ISR_BUDGET max_cycles=$maxTickCycles budget=7200"
    Write-Host "BINARY_TRAJ_STRESS PASS total=$Count"
    exit 0
} catch {
    Write-Host "BINARY_TRAJ_STRESS FAIL: $($_.Exception.Message)"
    try {
        if ($serial.IsOpen) {
            $serial.Write((New-Frame -Type $TYPE_ABORT -Seq 65535), 0, (New-Frame -Type $TYPE_ABORT -Seq 65535).Length)
        }
    } catch {}
    exit 1
} finally {
    if ($serial.IsOpen) { $serial.Close() }
}
