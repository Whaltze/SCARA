param(
    [string]$Port = "COM13",
    [int]$Baud = 115200,
    [ValidateSet("Car1", "Car2")]
    [string]$Shape = "Car1",
    [double]$StartX = 75.0,
    [double]$StartY = 345.3,
    [double]$CarX = 75.0,
    [double]$CarY = 200.0,
    [double]$FeedMmS = 20.0,
    [double]$SpacingMm = 0.35,
    [double]$MaxErrorMm = 1.0,
    [string]$CsvPath = "",
    [switch]$KeepEnabled,
    [switch]$UseZero,
    [switch]$RawProtocol
)

$ErrorActionPreference = "Stop"

if (-not $RawProtocol) {
    $root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..\..")).Path
    $python = "C:\Users\22602\.conda\envs\Robot\python.exe"
    if (-not (Test-Path -LiteralPath $python)) {
        $python = "python"
    }
    $shapeArg = $Shape.ToLowerInvariant()
    $argsList = @(
        (Join-Path $root "SCARA_UI\tests\ui_car_click_stress.py"),
        "--port", $Port,
        "--shape", $shapeArg,
        "--feed-mm-s", ([string]::Format([Globalization.CultureInfo]::InvariantCulture, "{0}", $FeedMmS)),
        "--car-x", ([string]::Format([Globalization.CultureInfo]::InvariantCulture, "{0}", $CarX)),
        "--car-y", ([string]::Format([Globalization.CultureInfo]::InvariantCulture, "{0}", $CarY)),
        "--max-error-mm", ([string]::Format([Globalization.CultureInfo]::InvariantCulture, "{0}", $MaxErrorMm))
    )
    if (-not [string]::IsNullOrWhiteSpace($CsvPath)) {
        $argsList += @("--csv-path", $CsvPath)
    }
    if ($UseZero) {
        $argsList += "--use-zero"
    }
    Write-Host "UI_BINARY_CAR_STRESS mode=ui_click"
    & $python @argsList
    exit $LASTEXITCODE
}

$SOF0 = 0xA5
$SOF1 = 0x5A
$VER = 1
$TYPE_BEGIN = 0x10
$TYPE_CHUNK = 0x11
$TYPE_VALIDATE = 0x12
$TYPE_RUN = 0x13
$TYPE_STATUS = 0x15
$TYPE_ACK = 0x80
$TYPE_NACK = 0x81
$TYPE_STATUS_RSP = 0x82
$PPR = 3200.0
$FLAG_EXACT_STOP = 0x0001
$FLAG_CARTESIAN_LINE = 0x0002
$ZERO1_MRAD = 2251.0
$ZERO2_MRAD = 890.0
$MRAD_PER_REV = 6283.0
$BASE_MM = 150.0
$ACTIVE_MM = 160.0
$PASSIVE_MM = 200.0
$CHUNK_SIZE = 20
$PRELOAD_POINTS = 100

$expected = New-Object System.Collections.Generic.List[object]
$samples = New-Object System.Collections.Generic.List[object]

function New-Point {
    param([double]$X, [double]$Y)
    return [pscustomobject]@{ X = $X; Y = $Y }
}

function Add-Point {
    param([System.Collections.Generic.List[object]]$List, [double]$X, [double]$Y)
    if ($List.Count -gt 0) {
        $last = $List[$List.Count - 1]
        if ([Math]::Sqrt(($X - $last.X) * ($X - $last.X) + ($Y - $last.Y) * ($Y - $last.Y)) -le 0.001) { return }
    }
    $List.Add((New-Point -X $X -Y $Y))
}

function Add-LinePoints {
    param([System.Collections.Generic.List[object]]$List, [double]$X0, [double]$Y0, [double]$X1, [double]$Y1, [double]$Spacing)
    $len = [Math]::Sqrt(($X1 - $X0) * ($X1 - $X0) + ($Y1 - $Y0) * ($Y1 - $Y0))
    $count = [Math]::Max(1, [int][Math]::Ceiling($len / [Math]::Max(0.05, $Spacing)))
    for ($i = 1; $i -le $count; $i++) {
        $t = [double]$i / [double]$count
        Add-Point -List $List -X ($X0 + ($X1 - $X0) * $t) -Y ($Y0 + ($Y1 - $Y0) * $t)
    }
}

function Get-ArcSpec {
    param([double]$X0, [double]$Y0, [double]$X1, [double]$Y1, [double]$Radius, [bool]$Clockwise)
    $dx = $X1 - $X0
    $dy = $Y1 - $Y0
    $chord = [Math]::Sqrt($dx * $dx + $dy * $dy)
    if ($chord -lt 0.001) { throw "arc start/end overlap" }
    $half = $chord * 0.5
    if ($Radius -lt $half) { throw "arc radius too small" }
    $mx = ($X0 + $X1) * 0.5
    $my = ($Y0 + $Y1) * 0.5
    $h = [Math]::Sqrt([Math]::Max(0.0, $Radius * $Radius - $half * $half))
    $nx = -$dy / $chord
    $ny = $dx / $chord
    $centers = @(
        (New-Point -X ($mx + $h * $nx) -Y ($my + $h * $ny)),
        (New-Point -X ($mx - $h * $nx) -Y ($my - $h * $ny))
    )
    $best = $null
    foreach ($c in $centers) {
        $a0 = [Math]::Atan2($Y0 - $c.Y, $X0 - $c.X)
        $a1 = [Math]::Atan2($Y1 - $c.Y, $X1 - $c.X)
        if ($Clockwise) { $delta = -(($a0 - $a1) % (2.0 * [Math]::PI)) }
        else { $delta = (($a1 - $a0) % (2.0 * [Math]::PI)) }
        if ([Math]::Abs($delta) -lt 1e-9) {
            if ($Clockwise) { $delta = -2.0 * [Math]::PI } else { $delta = 2.0 * [Math]::PI }
        }
        if ($null -eq $best -or [Math]::Abs($delta) -lt [Math]::Abs($best.Delta)) {
            $best = [pscustomobject]@{ Center = $c; A0 = $a0; Delta = $delta; Length = [Math]::Abs($delta) * $Radius }
        }
    }
    return $best
}

function Add-ArcPoints {
    param([System.Collections.Generic.List[object]]$List, [double]$X0, [double]$Y0, [double]$X1, [double]$Y1, [double]$Radius, [bool]$Clockwise, [double]$Spacing)
    $arc = Get-ArcSpec -X0 $X0 -Y0 $Y0 -X1 $X1 -Y1 $Y1 -Radius $Radius -Clockwise $Clockwise
    $count = [Math]::Max(8, [int][Math]::Ceiling($arc.Length / [Math]::Max(0.05, $Spacing)))
    for ($i = 1; $i -le $count; $i++) {
        $t = [double]$i / [double]$count
        $a = $arc.A0 + $arc.Delta * $t
        Add-Point -List $List -X ($arc.Center.X + $Radius * [Math]::Cos($a)) -Y ($arc.Center.Y + $Radius * [Math]::Sin($a))
    }
}

function Add-SendLine {
    param([System.Collections.Generic.List[object]]$List, [double]$X1, [double]$Y1)
    $List.Add([pscustomobject]@{
        X = $X1
        Y = $Y1
        Flags = ($FLAG_EXACT_STOP -bor $FLAG_CARTESIAN_LINE)
    })
}

function Add-SendArc {
    param([System.Collections.Generic.List[object]]$List, [double]$X0, [double]$Y0, [double]$X1, [double]$Y1, [double]$Radius, [bool]$Clockwise)
    $arc = Get-ArcSpec -X0 $X0 -Y0 $Y0 -X1 $X1 -Y1 $Y1 -Radius $Radius -Clockwise $Clockwise
    $count = [Math]::Max(2, [int][Math]::Ceiling($arc.Length / 0.35))
    for ($i = 1; $i -le $count; $i++) {
        $t = [double]$i / [double]$count
        $a = $arc.A0 + $arc.Delta * $t
        $flags = if ($i -eq $count) { $FLAG_EXACT_STOP } else { 0 }
        $List.Add([pscustomobject]@{
            X = $arc.Center.X + $Radius * [Math]::Cos($a)
            Y = $arc.Center.Y + $Radius * [Math]::Sin($a)
            Flags = $flags
        })
    }
}

function New-CarSendTargets {
    param([string]$Name, [double]$X0, [double]$Y0)
    $pts = New-Object System.Collections.Generic.List[object]
    Add-SendLine -List $pts -X1 $X0 -Y1 $Y0
    if ($Name -eq "Car1") {
        Add-SendLine -List $pts -X1 $X0 -Y1 ($Y0 + 24.0)
        Add-SendLine -List $pts -X1 ($X0 + 72.0) -Y1 ($Y0 + 24.0)
        Add-SendLine -List $pts -X1 ($X0 + 72.0) -Y1 ($Y0 + 48.0)
        Add-SendLine -List $pts -X1 ($X0 + 108.0) -Y1 ($Y0 + 48.0)
        Add-SendLine -List $pts -X1 ($X0 + 120.0) -Y1 ($Y0 + 36.0)
        Add-SendLine -List $pts -X1 ($X0 + 120.0) -Y1 $Y0
        Add-SendLine -List $pts -X1 ($X0 + 108.0) -Y1 $Y0
        Add-SendArc -List $pts -X0 ($X0 + 108.0) -Y0 $Y0 -X1 ($X0 + 84.0) -Y1 $Y0 -Radius 12.0 -Clockwise $true
        Add-SendLine -List $pts -X1 ($X0 + 48.0) -Y1 $Y0
        Add-SendArc -List $pts -X0 ($X0 + 48.0) -Y0 $Y0 -X1 ($X0 + 24.0) -Y1 $Y0 -Radius 12.0 -Clockwise $true
        Add-SendLine -List $pts -X1 $X0 -Y1 $Y0
    } else {
        Add-SendLine -List $pts -X1 $X0 -Y1 ($Y0 + 20.0)
        Add-SendLine -List $pts -X1 ($X0 + 40.0) -Y1 ($Y0 + 20.0)
        Add-SendLine -List $pts -X1 ($X0 + 60.0) -Y1 ($Y0 + 40.0)
        Add-SendLine -List $pts -X1 ($X0 + 120.0) -Y1 ($Y0 + 40.0)
        Add-SendLine -List $pts -X1 ($X0 + 140.0) -Y1 ($Y0 + 20.0)
        Add-SendLine -List $pts -X1 ($X0 + 160.0) -Y1 ($Y0 + 20.0)
        Add-SendLine -List $pts -X1 ($X0 + 160.0) -Y1 $Y0
        Add-SendLine -List $pts -X1 ($X0 + 140.0) -Y1 $Y0
        Add-SendArc -List $pts -X0 ($X0 + 140.0) -Y0 $Y0 -X1 ($X0 + 116.0) -Y1 $Y0 -Radius 12.0 -Clockwise $true
        Add-SendLine -List $pts -X1 ($X0 + 44.0) -Y1 $Y0
        Add-SendArc -List $pts -X0 ($X0 + 44.0) -Y0 $Y0 -X1 ($X0 + 20.0) -Y1 $Y0 -Radius 12.0 -Clockwise $true
        Add-SendLine -List $pts -X1 $X0 -Y1 $Y0
    }
    return $pts
}

function New-CarTargets {
    param([string]$Name, [double]$X0, [double]$Y0, [double]$Spacing)
    $pts = New-Object System.Collections.Generic.List[object]
    Add-LinePoints -List $pts -X0 $StartX -Y0 $StartY -X1 $X0 -Y1 $Y0 -Spacing $Spacing
    if ($Name -eq "Car1") {
        Add-LinePoints -List $pts -X0 $X0 -Y0 $Y0 -X1 $X0 -Y1 ($Y0 + 24.0) -Spacing $Spacing
        Add-LinePoints -List $pts -X0 $X0 -Y0 ($Y0 + 24.0) -X1 ($X0 + 72.0) -Y1 ($Y0 + 24.0) -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 72.0) -Y0 ($Y0 + 24.0) -X1 ($X0 + 72.0) -Y1 ($Y0 + 48.0) -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 72.0) -Y0 ($Y0 + 48.0) -X1 ($X0 + 108.0) -Y1 ($Y0 + 48.0) -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 108.0) -Y0 ($Y0 + 48.0) -X1 ($X0 + 120.0) -Y1 ($Y0 + 36.0) -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 120.0) -Y0 ($Y0 + 36.0) -X1 ($X0 + 120.0) -Y1 $Y0 -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 120.0) -Y0 $Y0 -X1 ($X0 + 108.0) -Y1 $Y0 -Spacing $Spacing
        Add-ArcPoints -List $pts -X0 ($X0 + 108.0) -Y0 $Y0 -X1 ($X0 + 84.0) -Y1 $Y0 -Radius 12.0 -Clockwise $true -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 84.0) -Y0 $Y0 -X1 ($X0 + 48.0) -Y1 $Y0 -Spacing $Spacing
        Add-ArcPoints -List $pts -X0 ($X0 + 48.0) -Y0 $Y0 -X1 ($X0 + 24.0) -Y1 $Y0 -Radius 12.0 -Clockwise $true -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 24.0) -Y0 $Y0 -X1 $X0 -Y1 $Y0 -Spacing $Spacing
    } else {
        Add-LinePoints -List $pts -X0 $X0 -Y0 $Y0 -X1 $X0 -Y1 ($Y0 + 20.0) -Spacing $Spacing
        Add-LinePoints -List $pts -X0 $X0 -Y0 ($Y0 + 20.0) -X1 ($X0 + 40.0) -Y1 ($Y0 + 20.0) -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 40.0) -Y0 ($Y0 + 20.0) -X1 ($X0 + 60.0) -Y1 ($Y0 + 40.0) -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 60.0) -Y0 ($Y0 + 40.0) -X1 ($X0 + 120.0) -Y1 ($Y0 + 40.0) -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 120.0) -Y0 ($Y0 + 40.0) -X1 ($X0 + 140.0) -Y1 ($Y0 + 20.0) -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 140.0) -Y0 ($Y0 + 20.0) -X1 ($X0 + 160.0) -Y1 ($Y0 + 20.0) -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 160.0) -Y0 ($Y0 + 20.0) -X1 ($X0 + 160.0) -Y1 $Y0 -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 160.0) -Y0 $Y0 -X1 ($X0 + 140.0) -Y1 $Y0 -Spacing $Spacing
        Add-ArcPoints -List $pts -X0 ($X0 + 140.0) -Y0 $Y0 -X1 ($X0 + 116.0) -Y1 $Y0 -Radius 12.0 -Clockwise $true -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 116.0) -Y0 $Y0 -X1 ($X0 + 44.0) -Y1 $Y0 -Spacing $Spacing
        Add-ArcPoints -List $pts -X0 ($X0 + 44.0) -Y0 $Y0 -X1 ($X0 + 20.0) -Y1 $Y0 -Radius 12.0 -Clockwise $true -Spacing $Spacing
        Add-LinePoints -List $pts -X0 ($X0 + 20.0) -Y0 $Y0 -X1 $X0 -Y1 $Y0 -Spacing $Spacing
    }
    return $pts
}

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
    for ($i = 0; $i -lt $Length; $i++) { $crc = Update-Crc16 -Crc $crc -Byte $Data[$Offset + $i] }
    return $crc
}

function Write-U16 {
    param([System.Collections.Generic.List[byte]]$Bytes, [int]$Value)
    $Bytes.Add([byte]($Value -band 0xFF)); $Bytes.Add([byte](($Value -shr 8) -band 0xFF))
}

function Write-U32 {
    param([System.Collections.Generic.List[byte]]$Bytes, [long]$Value)
    $Bytes.Add([byte]($Value -band 0xFF)); $Bytes.Add([byte](($Value -shr 8) -band 0xFF))
    $Bytes.Add([byte](($Value -shr 16) -band 0xFF)); $Bytes.Add([byte](($Value -shr 24) -band 0xFF))
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
        if ($buf.Count -eq 0) { if ($b -eq $SOF0) { $buf.Add([byte]$b) }; continue }
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
            $payload = if ($len -gt 0) { $arr[8..(7 + $len)] } else { @() }
            return [pscustomobject]@{
                Type = $arr[3]
                Seq = [BitConverter]::ToUInt16($arr, 4)
                Payload = $payload
            }
        }
    }
    throw "binary frame timeout"
}

function Write-FrameBytes {
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
}

function Send-Frame {
    param([System.IO.Ports.SerialPort]$Serial, [int]$Type, [int]$Seq, [byte[]]$Payload = @())
    Write-FrameBytes -Serial $Serial -Type $Type -Seq $Seq -Payload $Payload
    $rsp = Read-Frame -Serial $Serial
    if ($rsp.Seq -ne ($Seq -band 0xFFFF)) { throw "seq mismatch tx=$Seq rx=$($rsp.Seq)" }
    if ($rsp.Type -eq $TYPE_NACK) {
        $err = if ($rsp.Payload.Length -gt 1) { $rsp.Payload[1] } else { 255 }
        throw ("NACK type=0x{0:X2} err={1}" -f $Type, $err)
    }
    if ($rsp.Type -ne $TYPE_ACK) { throw ("unexpected response type=0x{0:X2}" -f $rsp.Type) }
}

function Send-BinaryStatus {
    param([System.IO.Ports.SerialPort]$Serial, [int]$Seq)
    Write-FrameBytes -Serial $Serial -Type $TYPE_STATUS -Seq $Seq -Payload @()
    $rsp = Read-Frame -Serial $Serial
    if ($rsp.Type -ne $TYPE_STATUS_RSP) { throw ("unexpected status response type=0x{0:X2}" -f $rsp.Type) }
    if ($rsp.Payload.Length -lt 19) { throw "short binary status payload" }
    return [pscustomobject]@{
        Queued = [BitConverter]::ToUInt16($rsp.Payload, 2)
        Free = [BitConverter]::ToUInt16($rsp.Payload, 4)
        Accepted = [BitConverter]::ToUInt32($rsp.Payload, 6)
        Executed = [BitConverter]::ToUInt32($rsp.Payload, 10)
        Total = [BitConverter]::ToUInt32($rsp.Payload, 14)
        State = $rsp.Payload[18]
    }
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

function Remove-ZeroPulseTargets {
    param([object[]]$Points)
    $filtered = New-Object System.Collections.Generic.List[object]
    $prevPulse = Convert-UiToPulse -X $StartX -Y $StartY
    foreach ($point in $Points) {
        $pulse = Convert-UiToPulse -X $point.X -Y $point.Y
        $dom = [Math]::Max([Math]::Abs($pulse.P1 - $prevPulse.P1), [Math]::Abs($pulse.P2 - $prevPulse.P2))
        if ($dom -gt 0) {
            $filtered.Add($point)
            $prevPulse = $pulse
        }
    }
    return $filtered
}

function New-PointPayload {
    param([object[]]$Targets, [int]$Offset, [int]$Count)
    $bytes = New-Object System.Collections.Generic.List[byte]
    if ($Offset -eq 0) {
        $prev = New-Point -X $StartX -Y $StartY
    } else {
        $prev = $Targets[$Offset - 1]
    }
    $prevPulse = Convert-UiToPulse -X $prev.X -Y $prev.Y
    for ($i = $Offset; $i -lt ($Offset + $Count); $i++) {
        $target = $Targets[$i]
        $flags = if ($target.PSObject.Properties.Name -contains "Flags") { [int]$target.Flags } else { 0 }
        if (($flags -band $FLAG_CARTESIAN_LINE) -ne 0) {
            $xUm = [int][Math]::Round(($target.X - $BASE_MM * 0.5) * 1000.0)
            $yUm = [int][Math]::Round($target.Y * 1000.0)
            $feedMmMin = [int][Math]::Round($FeedMmS * 60.0)
            Write-I32 -Bytes $bytes -Value $xUm
            Write-I32 -Bytes $bytes -Value $yUm
            Write-U16 -Bytes $bytes -Value $feedMmMin
            Write-U16 -Bytes $bytes -Value $flags
            $prev = $target
            $prevPulse = Convert-UiToPulse -X $target.X -Y $target.Y
            continue
        }
        $pulse = Convert-UiToPulse -X $target.X -Y $target.Y
        $dist = [Math]::Sqrt(($target.X - $prev.X) * ($target.X - $prev.X) + ($target.Y - $prev.Y) * ($target.Y - $prev.Y))
        $dom = [Math]::Max([Math]::Abs($pulse.P1 - $prevPulse.P1), [Math]::Abs($pulse.P2 - $prevPulse.P2))
        if ($dom -eq 0) { continue }
        $duration = $dist / [Math]::Max(0.1, $FeedMmS)
        $vdom = [int][Math]::Ceiling($dom / [Math]::Max(0.001, $duration))
        if ($vdom -lt 16) { $vdom = 16 }
        if ($vdom -gt 10000) { $vdom = 10000 }
        Write-I32 -Bytes $bytes -Value $pulse.P1
        Write-I32 -Bytes $bytes -Value $pulse.P2
        Write-U16 -Bytes $bytes -Value $vdom
        Write-U16 -Bytes $bytes -Value $flags
        $prev = $target
        $prevPulse = $pulse
    }
    return $bytes.ToArray()
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

function Add-FeedbackSample {
    param([double]$X, [double]$Y)
    $best = $null
    for ($i = 0; $i -lt ($script:expected.Count - 1); $i++) {
        $a = $script:expected[$i]
        $b = $script:expected[$i + 1]
        $err = Get-LineError -Px $X -Py $Y -Ax $a.X -Ay $a.Y -Bx $b.X -By $b.Y
        if ($null -eq $best -or $err -lt $best.Error) {
            $best = [pscustomobject]@{ Error = $err }
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

function Get-PathLength {
    param([object[]]$Points)
    $length = 0.0
    for ($i = 1; $i -lt $Points.Count; $i++) {
        $a = $Points[$i - 1]
        $b = $Points[$i]
        $length += [Math]::Sqrt(($b.X - $a.X) * ($b.X - $a.X) + ($b.Y - $a.Y) * ($b.Y - $a.Y))
    }
    return $length
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
    $rawTargets = @(New-CarTargets -Name $Shape -X0 $CarX -Y0 $CarY -Spacing $SpacingMm)
    foreach ($p in $rawTargets) { $expected.Add($p) }
    $targets = @(Remove-ZeroPulseTargets -Points @(New-CarSendTargets -Name $Shape -X0 $CarX -Y0 $CarY))
    Write-Host ("UI_BINARY_CAR shape={0} keypoints={1} preview={2} mode=cartesian_lines start=({3:F3},{4:F3}) car=({5:F3},{6:F3}) feed={7:F3}mm/s spacing={8:F3}" -f $Shape, $targets.Count, $rawTargets.Count, $StartX, $StartY, $CarX, $CarY, $FeedMmS, $SpacingMm)
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
    $sent = 0
    $preload = [Math]::Min($targets.Count, $PRELOAD_POINTS)
    while ($sent -lt $preload) {
        $take = [Math]::Min($CHUNK_SIZE, $preload - $sent)
        Send-Frame -Serial $serial -Type $TYPE_CHUNK -Seq $seq -Payload (New-PointPayload -Targets $targets -Offset $sent -Count $take); $seq++
        $sent += $take
    }
    Send-Frame -Serial $serial -Type $TYPE_VALIDATE -Seq $seq; $seq++
    Send-Frame -Serial $serial -Type $TYPE_RUN -Seq $seq; $seq++
    while ($sent -lt $targets.Count) {
        $st = Send-BinaryStatus -Serial $serial -Seq $seq; $seq++
        if ($st.Free -le 0) { Start-Sleep -Milliseconds 20; continue }
        $take = [Math]::Min($CHUNK_SIZE, [Math]::Min($st.Free, $targets.Count - $sent))
        Send-Frame -Serial $serial -Type $TYPE_CHUNK -Seq $seq -Payload (New-PointPayload -Targets $targets -Offset $sent -Count $take); $seq++
        $sent += $take
    }

    $pathLen = Get-PathLength -Points $rawTargets
    $motionSeconds = $pathLen / [Math]::Max(0.1, $FeedMmS)
    $deadline = (Get-Date).AddSeconds([Math]::Max(45.0, $motionSeconds * 2.4 + 20.0))
    $idleSeen = $false
    while ((Get-Date) -lt $deadline) {
        $serial.Write("?")
        $pollUntil = (Get-Date).AddMilliseconds(180)
        while ((Get-Date) -lt $pollUntil) {
            try { $line = $serial.ReadLine().Trim() } catch [TimeoutException] { continue }
            if ($line.StartsWith("<")) {
                Parse-Status -Line $line
                if ($line -match "^<Idle" -and $line -match "JT:(Done|Idle)" -and $line -match "E:0") { $idleSeen = $true }
            }
        }
        if ($idleSeen) { break }
    }
    if (-not $idleSeen) { throw "motion did not finish cleanly" }
    if (-not $KeepEnabled) { Send-Ascii -Serial $serial -Line "ENABLE 0" | Out-Null }

    $summary = Get-Summary
    Export-CsvIfNeeded
    Write-Host ("UI_BINARY_CAR_ERROR samples={0} max={1:F4} rms={2:F4}" -f $summary.Count, $summary.Max, $summary.Rms)
    if ($summary.Count -lt 3) { throw "too few feedback samples: $($summary.Count)" }
    if ($summary.Max -gt $MaxErrorMm) { throw ("max error {0:F4}mm > {1:F4}mm" -f $summary.Max, $MaxErrorMm) }
    Write-Host "UI_BINARY_CAR_STRESS PASS"
    exit 0
} catch {
    Write-Host "UI_BINARY_CAR_STRESS FAIL: $($_.Exception.Message)"
    exit 1
} finally {
    if ($serial.IsOpen) { $serial.Close() }
}
