param(
    [string]$Port = "COM13",
    [int]$Baud = 115200,
    [double]$StartX = 75.0,
    [double]$StartY = 345.3,
    [double]$EndX = 150.0,
    [double]$EndY = 250.0,
    [double]$FeedMmS = 20.0,
    [double]$MaxErrorMm = 1.0,
    [string]$CsvPath = "",
    [switch]$KeepEnabled
)

$ErrorActionPreference = "Stop"

$SOF0 = 0xA5
$SOF1 = 0x5A
$VER = 1
$TYPE_BEGIN = 0x10
$TYPE_CHUNK = 0x11
$TYPE_VALIDATE = 0x12
$TYPE_RUN = 0x13
$TYPE_ACK = 0x80
$TYPE_NACK = 0x81
$PPR = 3200.0
$FLAG_EXACT_STOP = 0x0001
$FLAG_CARTESIAN_LINE = 0x0002
$ZERO1_MRAD = 2251.0
$ZERO2_MRAD = 890.0
$MRAD_PER_REV = 6283.0
$BASE_MM = 150.0
$ACTIVE_MM = 160.0
$PASSIVE_MM = 200.0
$LINE_TOLERANCE_MM = 0.90
$LINE_MAX_SEGMENT_MM = 10.0
$expected = New-Object System.Collections.Generic.List[object]
$samples = New-Object System.Collections.Generic.List[object]

function Update-Crc16 {
    param([int]$Crc, [int]$Byte)
    $crc = $Crc -bxor ($Byte -band 0xFF)
    for ($i = 0; $i -lt 8; $i++) {
        if (($crc -band 1) -ne 0) { $crc = (($crc -shr 1) -bxor 0xA001) -band 0xFFFF }
        else { $crc = ($crc -shr 1) -band 0xFFFF }
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
    foreach ($b in [System.BitConverter]::GetBytes([int]$Value)) { $Bytes.Add($b) }
}

function Read-Frame {
    param([System.IO.Ports.SerialPort]$Serial, [int]$TimeoutMs = 1500)
    $deadline = (Get-Date).AddMilliseconds($TimeoutMs)
    $buf = New-Object System.Collections.Generic.List[byte]
    $expectedLen = $null
    while ((Get-Date) -lt $deadline) {
        try { $b = $Serial.ReadByte() } catch [TimeoutException] { continue }
        if ($b -lt 0) { continue }
        if ($buf.Count -eq 0) {
            if ($b -eq $SOF0) { $buf.Add([byte]$b) }
            continue
        }
        if ($buf.Count -eq 1) {
            if ($b -eq $SOF1) { $buf.Add([byte]$b) }
            elseif ($b -eq $SOF0) { $buf.Clear(); $buf.Add([byte]$b) }
            else { $buf.Clear() }
            continue
        }
        $buf.Add([byte]$b)
        if ($buf.Count -eq 8 -and $null -eq $expectedLen) {
            $payloadLen = [BitConverter]::ToUInt16($buf.ToArray(), 6)
            $expectedLen = 10 + $payloadLen
        }
        if ($null -ne $expectedLen -and $buf.Count -ge $expectedLen) {
            $arr = $buf.ToArray()
            $len = [BitConverter]::ToUInt16($arr, 6)
            $rxCrc = [BitConverter]::ToUInt16($arr, 8 + $len)
            $calc = Get-Crc16 -Data $arr -Offset 2 -Length (6 + $len)
            if ($rxCrc -ne $calc) { throw "CRC mismatch rx=$rxCrc calc=$calc" }
            return [pscustomobject]@{
                Ver = $arr[2]
                Type = $arr[3]
                Seq = [BitConverter]::ToUInt16($arr, 4)
                Payload = $arr[8..(7 + $len)]
            }
        }
    }
    throw "binary frame timeout"
}

function Send-Frame {
    param([System.IO.Ports.SerialPort]$Serial, [int]$Type, [int]$Seq, [byte[]]$Payload = @())
    $bytes = New-Object System.Collections.Generic.List[byte]
    $bytes.Add([byte]$SOF0); $bytes.Add([byte]$SOF1)
    $bytes.Add([byte]$VER); $bytes.Add([byte]$Type)
    Write-U16 -Bytes $bytes -Value $Seq
    Write-U16 -Bytes $bytes -Value $Payload.Length
    foreach ($b in $Payload) { $bytes.Add($b) }
    $arr = $bytes.ToArray()
    $crc = Get-Crc16 -Data $arr -Offset 2 -Length ($arr.Length - 2)
    Write-U16 -Bytes $bytes -Value $crc
    $out = $bytes.ToArray()
    $Serial.Write($out, 0, $out.Length)
    $rsp = Read-Frame -Serial $Serial
    if ($rsp.Seq -ne ($Seq -band 0xFFFF)) { throw "seq mismatch tx=$Seq rx=$($rsp.Seq)" }
    if ($rsp.Type -eq $TYPE_NACK) {
        $err = if ($rsp.Payload.Length -gt 1) { $rsp.Payload[1] } else { 255 }
        throw ("NACK type=0x{0:X2} err={1}" -f $Type, $err)
    }
    if ($rsp.Type -ne $TYPE_ACK) { throw ("unexpected response type=0x{0:X2}" -f $rsp.Type) }
}

function Send-Ascii {
    param([System.IO.Ports.SerialPort]$Serial, [string]$Line, [string]$Prefix = "OK")
    $Serial.Write($Line + "`n")
    $deadline = (Get-Date).AddMilliseconds(1800)
    while ((Get-Date) -lt $deadline) {
        try { $rx = $Serial.ReadLine().Trim() } catch [TimeoutException] { continue }
        if ($rx.StartsWith("<")) { Parse-Status -Line $rx; continue }
        if ($rx.StartsWith($Prefix)) { Write-Host "RX $rx"; return $rx }
        if ($rx.ToLower().Contains("error:")) { throw $rx }
    }
    throw "timeout waiting '$Prefix' for '$Line'"
}

function Get-Inverse {
    param([double]$X, [double]$Y)
    $d1 = [Math]::Sqrt($X * $X + $Y * $Y)
    $d2 = [Math]::Sqrt(($X - $BASE_MM) * ($X - $BASE_MM) + $Y * $Y)
    $a1 = [Math]::Acos([Math]::Max(-1.0, [Math]::Min(1.0, (($ACTIVE_MM * $ACTIVE_MM + $d1 * $d1 - $PASSIVE_MM * $PASSIVE_MM) / (2.0 * $ACTIVE_MM * $d1)))))
    $a2 = [Math]::Acos([Math]::Max(-1.0, [Math]::Min(1.0, (($ACTIVE_MM * $ACTIVE_MM + $d2 * $d2 - $PASSIVE_MM * $PASSIVE_MM) / (2.0 * $ACTIVE_MM * $d2)))))
    $q1 = [Math]::Atan2($Y, $X) + $a1
    $q2 = [Math]::Atan2($Y, $X - $BASE_MM) - $a2
    return [pscustomobject]@{ Q1 = $q1 * 180.0 / [Math]::PI; Q2 = $q2 * 180.0 / [Math]::PI }
}

function Convert-UiToPulse {
    param([double]$X, [double]$Y)
    $q = Get-Inverse -X $X -Y $Y
    $m1 = [int]([Math]::Truncate(($q.Q1 * [Math]::PI / 180.0) * 1000.0))
    $m2 = [int]([Math]::Truncate(($q.Q2 * [Math]::PI / 180.0) * 1000.0))
    $p1 = [int]((($m1 - [int]$ZERO1_MRAD) * $PPR) / $MRAD_PER_REV)
    $p2 = [int]((($m2 - [int]$ZERO2_MRAD) * $PPR) / $MRAD_PER_REV)
    return [pscustomobject]@{ P1 = $p1; P2 = $p2 }
}

function Convert-PulseToUi {
    param([int]$P1, [int]$P2)
    $theta1Mrad = [int]([Math]::Truncate(($P1 * $MRAD_PER_REV) / $PPR)) + [int]$ZERO1_MRAD
    $theta2Mrad = [int]([Math]::Truncate(($P2 * $MRAD_PER_REV) / $PPR)) + [int]$ZERO2_MRAD
    $t1 = $theta1Mrad / 1000.0
    $t2 = $theta2Mrad / 1000.0
    $ex1 = $ACTIVE_MM * [Math]::Cos($t1)
    $ey1 = $ACTIVE_MM * [Math]::Sin($t1)
    $ex2 = $BASE_MM + $ACTIVE_MM * [Math]::Cos($t2)
    $ey2 = $ACTIVE_MM * [Math]::Sin($t2)
    $dx = $ex2 - $ex1
    $dy = $ey2 - $ey1
    $d = [Math]::Sqrt($dx * $dx + $dy * $dy)
    $mx = ($ex1 + $ex2) * 0.5
    $my = ($ey1 + $ey2) * 0.5
    $h = [Math]::Sqrt([Math]::Max(0.0, $PASSIVE_MM * $PASSIVE_MM - ($d * 0.5) * ($d * 0.5)))
    $ux = -$dy / $d
    $uy = $dx / $d
    $ix1 = $mx + $ux * $h
    $iy1 = $my + $uy * $h
    $ix2 = $mx - $ux * $h
    $iy2 = $my - $uy * $h
    if ($iy1 -ge $iy2) { return [pscustomobject]@{ X = $ix1; Y = $iy1 } }
    return [pscustomobject]@{ X = $ix2; Y = $iy2 }
}

function Get-LineError {
    param([double]$Px, [double]$Py, [double]$Ax, [double]$Ay, [double]$Bx, [double]$By)
    $vx = $Bx - $Ax
    $vy = $By - $Ay
    $den = $vx * $vx + $vy * $vy
    if ($den -le 1e-12) { return [Math]::Sqrt(($Px - $Ax) * ($Px - $Ax) + ($Py - $Ay) * ($Py - $Ay)) }
    $t = (($Px - $Ax) * $vx + ($Py - $Ay) * $vy) / $den
    if ($t -lt 0.0) { $t = 0.0 }
    if ($t -gt 1.0) { $t = 1.0 }
    $qx = $Ax + $vx * $t
    $qy = $Ay + $vy * $t
    return [Math]::Sqrt(($Px - $qx) * ($Px - $qx) + ($Py - $qy) * ($Py - $qy))
}

function Get-JointLinearizedError {
    param([object]$A, [object]$B)
    $pa = Convert-UiToPulse -X $A.X -Y $A.Y
    $pb = Convert-UiToPulse -X $B.X -Y $B.Y
    $max = 0.0
    foreach ($t in @(0.25, 0.5, 0.75)) {
        $p1 = [int][Math]::Round($pa.P1 + ($pb.P1 - $pa.P1) * $t)
        $p2 = [int][Math]::Round($pa.P2 + ($pb.P2 - $pa.P2) * $t)
        $xy = Convert-PulseToUi -P1 $p1 -P2 $p2
        $err = Get-LineError -Px $xy.X -Py $xy.Y -Ax $A.X -Ay $A.Y -Bx $B.X -By $B.Y
        if ($err -gt $max) { $max = $err }
    }
    return $max
}

function Get-AdaptiveLineTargets {
    param([object]$A, [object]$B, [int]$Depth = 0)
    $len = [Math]::Sqrt(($B.X - $A.X) * ($B.X - $A.X) + ($B.Y - $A.Y) * ($B.Y - $A.Y))
    $err = Get-JointLinearizedError -A $A -B $B
    if ($Depth -ge 12 -or ($len -le $LINE_MAX_SEGMENT_MM -and $err -le $LINE_TOLERANCE_MM)) {
        return @($B)
    }
    $mid = [pscustomobject]@{ X = ($A.X + $B.X) * 0.5; Y = ($A.Y + $B.Y) * 0.5 }
    return @((Get-AdaptiveLineTargets -A $A -B $mid -Depth ($Depth + 1))) + @((Get-AdaptiveLineTargets -A $mid -B $B -Depth ($Depth + 1)))
}

function New-PointPayload {
    param([object[]]$Targets)
    $bytes = New-Object System.Collections.Generic.List[byte]
    $prev = [pscustomobject]@{ X = $StartX; Y = $StartY }
    $prevPulse = Convert-UiToPulse -X $prev.X -Y $prev.Y
    foreach ($target in $Targets) {
        $xUm = [int][Math]::Round(($target.X - $BASE_MM * 0.5) * 1000.0)
        $yUm = [int][Math]::Round($target.Y * 1000.0)
        $feedMmMin = [int][Math]::Round($FeedMmS * 60.0)
        Write-I32 -Bytes $bytes -Value $xUm
        Write-I32 -Bytes $bytes -Value $yUm
        Write-U16 -Bytes $bytes -Value $feedMmMin
        Write-U16 -Bytes $bytes -Value ($FLAG_EXACT_STOP -bor $FLAG_CARTESIAN_LINE)
        $prev = $target
    }
    return $bytes.ToArray()
}

function Add-ExpectedLine {
    $steps = [Math]::Max(2, [int][Math]::Ceiling([Math]::Sqrt(($EndX - $StartX) * ($EndX - $StartX) + ($EndY - $StartY) * ($EndY - $StartY)) / 0.25))
    for ($i = 0; $i -le $steps; $i++) {
        $t = [double]$i / [double]$steps
        $script:expected.Add([pscustomobject]@{ X = $StartX + ($EndX - $StartX) * $t; Y = $StartY + ($EndY - $StartY) * $t })
    }
}

function Add-FeedbackSample {
    param([double]$X, [double]$Y)
    $best = $null
    for ($i = 0; $i -lt ($script:expected.Count - 1); $i++) {
        $a = $script:expected[$i]
        $b = $script:expected[$i + 1]
        $err = Get-LineError -Px $X -Py $Y -Ax $a.X -Ay $a.Y -Bx $b.X -By $b.Y
        if ($null -eq $best -or $err -lt $best.Error) {
            $best = [pscustomobject]@{ ExpectedX = $a.X; ExpectedY = $a.Y; Error = $err }
        }
    }
    if ($null -ne $best) {
        $script:samples.Add([pscustomobject]@{ Index = $script:samples.Count + 1; X = $X; Y = $Y; Error = $best.Error })
    }
}

function Parse-Status {
    param([string]$Line)
    if ($Line -match "M:([-0-9.]+),([-0-9.]+)") {
        Add-FeedbackSample -X (([double]$Matches[1]) + $BASE_MM * 0.5) -Y ([double]$Matches[2])
    }
}

function Get-Summary {
    if ($samples.Count -eq 0) { return [pscustomobject]@{ Count = 0; Max = 0.0; Rms = 0.0 } }
    $max = 0.0
    $sum2 = 0.0
    foreach ($s in $samples) {
        if ($s.Error -gt $max) { $max = $s.Error }
        $sum2 += $s.Error * $s.Error
    }
    return [pscustomobject]@{ Count = $samples.Count; Max = $max; Rms = [Math]::Sqrt($sum2 / $samples.Count) }
}

function Export-CsvIfNeeded {
    if ([string]::IsNullOrWhiteSpace($CsvPath)) { return }
    $dir = Split-Path -Parent $CsvPath
    if (-not [string]::IsNullOrWhiteSpace($dir) -and -not (Test-Path -LiteralPath $dir)) { New-Item -ItemType Directory -Path $dir | Out-Null }
    $samples | Export-Csv -LiteralPath $CsvPath -NoTypeInformation -Encoding UTF8
    Write-Host "CSV $CsvPath"
}

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 100
$serial.WriteTimeout = 1000

try {
    Add-ExpectedLine
    $targets = @([pscustomobject]@{ X = $EndX; Y = $EndY })
    Write-Host ("UI_BINARY_LINE keypoints={0} mode=cartesian_endpoint start=({1:F3},{2:F3}) end=({3:F3},{4:F3}) feed={5:F3}mm/s" -f $targets.Count, $StartX, $StartY, $EndX, $EndY, $FeedMmS)
    $serial.Open()
    Start-Sleep -Milliseconds 300
    while ($serial.BytesToRead -gt 0) { try { [void]$serial.ReadLine() } catch { break } }
    Send-Ascii -Serial $serial -Line "VERSION" | Out-Null
    Send-Ascii -Serial $serial -Line "HOSTCAP" | Out-Null
    Send-Ascii -Serial $serial -Line "PPR 3200 3200" | Out-Null
    Send-Ascii -Serial $serial -Line "CLEAR_ERROR" | Out-Null
    Send-Ascii -Serial $serial -Line "ZERO" | Out-Null
    Send-Ascii -Serial $serial -Line "ENABLE 1" | Out-Null

    $seq = 1
    $begin = New-Object System.Collections.Generic.List[byte]
    Write-U32 -Bytes $begin -Value $targets.Count
    Send-Frame -Serial $serial -Type $TYPE_BEGIN -Seq $seq -Payload $begin.ToArray(); $seq++
    Send-Frame -Serial $serial -Type $TYPE_CHUNK -Seq $seq -Payload (New-PointPayload -Targets $targets); $seq++
    Send-Frame -Serial $serial -Type $TYPE_VALIDATE -Seq $seq; $seq++
    Send-Frame -Serial $serial -Type $TYPE_RUN -Seq $seq; $seq++

    $pathLen = [Math]::Sqrt(($EndX - $StartX) * ($EndX - $StartX) + ($EndY - $StartY) * ($EndY - $StartY))
    $motionSeconds = $pathLen / [Math]::Max(0.1, $FeedMmS)
    $deadline = (Get-Date).AddSeconds([Math]::Max(20.0, $motionSeconds * 1.8 + 10.0))
    $idleSeen = $false
    while ((Get-Date) -lt $deadline) {
        $serial.Write("?")
        $pollUntil = (Get-Date).AddMilliseconds(180)
        while ((Get-Date) -lt $pollUntil) {
            try { $line = $serial.ReadLine().Trim() } catch [TimeoutException] { continue }
            if ($line.StartsWith("<")) {
                Parse-Status -Line $line
                if ($line -match "^<Idle" -and $line -match "JT:(Done|Idle)" -and $line -match "E:0") {
                    $idleSeen = $true
                }
            }
        }
        if ($idleSeen) { break }
    }
    if (-not $idleSeen) { throw "motion did not finish cleanly" }
    if (-not $KeepEnabled) { Send-Ascii -Serial $serial -Line "ENABLE 0" | Out-Null }

    $summary = Get-Summary
    Export-CsvIfNeeded
    Write-Host ("UI_BINARY_LINE_ERROR samples={0} max={1:F4} rms={2:F4}" -f $summary.Count, $summary.Max, $summary.Rms)
    if ($summary.Count -lt 3) { throw "too few feedback samples: $($summary.Count)" }
    if ($summary.Max -gt $MaxErrorMm) { throw ("max error {0:F4}mm > {1:F4}mm" -f $summary.Max, $MaxErrorMm) }
    Write-Host "UI_BINARY_LINE_STRESS PASS"
    exit 0
} catch {
    Write-Host "UI_BINARY_LINE_STRESS FAIL: $($_.Exception.Message)"
    exit 1
} finally {
    if ($serial.IsOpen) { $serial.Close() }
}
