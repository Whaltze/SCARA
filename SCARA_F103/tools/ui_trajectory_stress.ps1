param(
    [string]$Port = "AUTO",
    [int]$Baud = 115200,
    [int]$TimeoutMs = 5000,
    [int]$CommandTimeoutMs = 60000,
    [int]$DrainTimeoutMs = 120000,
    [int]$Count = 3000,
    [double]$FeedMmMin = 900.0,
    [switch]$KeepEnabled
)

# UI trajectory stress:
# Replays the upper-computer trajectory button logic with one G1 line,
# one G2 clockwise arc, one G3 counter-clockwise arc, and two fixed car
# outlines. The firmware only receives host-planned G1 points.

$ErrorActionPreference = "Stop"

$BaseMm = 150.0
$ActiveMm = 160.0
$PassiveMm = 200.0
$MinY = 10.0
$MinAnchorDist = 40.0
$MaxAnchorDist = 360.0
$Theta1Min = -180.0
$Theta1Max = 180.0
$Theta2Min = 0.0
$Theta2Max = 360.0

function Resolve-SerialPortName {
    param([string]$Requested)
    if ($Requested -and $Requested.ToUpperInvariant() -ne "AUTO") {
        return $Requested
    }
    $ports = @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)
    if ($ports.Count -eq 1) {
        Write-Host ("AUTO port selected: {0}" -f $ports[0])
        return $ports[0]
    }
    if ($ports.Count -eq 0) {
        throw "No serial port found. Use -Port COMx after connecting the controller."
    }
    throw ("Multiple serial ports found: {0}. Use -Port COMx." -f ($ports -join ", "))
}

function Get-Checksum {
    param([string]$Line)
    $sum = 0
    foreach ($b in [System.Text.Encoding]::ASCII.GetBytes($Line)) {
        $sum = ($sum + $b) -band 0xFF
    }
    return "{0:X2}" -f $sum
}

function Read-Line-Safe {
    param([System.IO.Ports.SerialPort]$Serial)
    try {
        return $Serial.ReadLine().Trim()
    } catch [TimeoutException] {
        return "<timeout>"
    }
}

function Convert-UiToMcuX {
    param([double]$X)
    return $X - $BaseMm * 0.5
}

function Get-InverseDeg {
    param([double]$X, [double]$Y)
    $d1 = [Math]::Sqrt($X * $X + $Y * $Y)
    $d2 = [Math]::Sqrt(($X - $BaseMm) * ($X - $BaseMm) + $Y * $Y)
    if ($d1 -le 0.001 -or $d2 -le 0.001) { throw "IK distance too small" }
    $c1 = (($ActiveMm * $ActiveMm) + ($d1 * $d1) - ($PassiveMm * $PassiveMm)) / (2.0 * $ActiveMm * $d1)
    $c2 = (($ActiveMm * $ActiveMm) + ($d2 * $d2) - ($PassiveMm * $PassiveMm)) / (2.0 * $ActiveMm * $d2)
    $c1 = [Math]::Max(-1.0, [Math]::Min(1.0, $c1))
    $c2 = [Math]::Max(-1.0, [Math]::Min(1.0, $c2))
    $q1 = [Math]::Atan2($Y, $X) + [Math]::Acos($c1)
    $q2 = [Math]::Atan2($Y, $X - $BaseMm) - [Math]::Acos($c2)
    return [pscustomobject]@{
        Q1 = $q1 * 180.0 / [Math]::PI
        Q2 = $q2 * 180.0 / [Math]::PI
    }
}

function Get-Violations {
    param([double]$X, [double]$Y, [int]$Index, [string]$Name)
    $items = New-Object System.Collections.Generic.List[string]
    $d1 = [Math]::Sqrt($X * $X + $Y * $Y)
    $d2 = [Math]::Sqrt(($X - $BaseMm) * ($X - $BaseMm) + $Y * $Y)
    if ($Y -lt $MinY) { $items.Add("$Name point $Index Y below min by $([Math]::Round($MinY - $Y, 3)) mm") }
    if ($d1 -lt $MinAnchorDist) { $items.Add("$Name point $Index left anchor below min by $([Math]::Round($MinAnchorDist - $d1, 3)) mm") }
    if ($d1 -gt $MaxAnchorDist) { $items.Add("$Name point $Index left anchor above max by $([Math]::Round($d1 - $MaxAnchorDist, 3)) mm") }
    if ($d2 -lt $MinAnchorDist) { $items.Add("$Name point $Index right anchor below min by $([Math]::Round($MinAnchorDist - $d2, 3)) mm") }
    if ($d2 -gt $MaxAnchorDist) { $items.Add("$Name point $Index right anchor above max by $([Math]::Round($d2 - $MaxAnchorDist, 3)) mm") }

    try {
        $q = Get-InverseDeg -X $X -Y $Y
    } catch {
        $items.Add("$Name point $Index IK failed at X=$X Y=$Y")
        return $items
    }

    if ($q.Q1 -lt $Theta1Min) { $items.Add("$Name point $Index M1 below min by $([Math]::Round($Theta1Min - $q.Q1, 3)) deg") }
    if ($q.Q1 -gt $Theta1Max) { $items.Add("$Name point $Index M1 above max by $([Math]::Round($q.Q1 - $Theta1Max, 3)) deg") }
    if ($q.Q2 -lt $Theta2Min) { $items.Add("$Name point $Index M2 below min by $([Math]::Round($Theta2Min - $q.Q2, 3)) deg") }
    if ($q.Q2 -gt $Theta2Max) { $items.Add("$Name point $Index M2 above max by $([Math]::Round($q.Q2 - $Theta2Max, 3)) deg") }

    $q1 = $q.Q1 * [Math]::PI / 180.0
    $q2 = $q.Q2 * [Math]::PI / 180.0
    $leftX = $ActiveMm * [Math]::Cos($q1)
    $leftY = $ActiveMm * [Math]::Sin($q1)
    $rightX = $BaseMm + $ActiveMm * [Math]::Cos($q2)
    $rightY = $ActiveMm * [Math]::Sin($q2)
    if ($leftX -ge $rightX) { $items.Add("$Name point $Index active arms cross by $([Math]::Round($leftX - $rightX, 3)) mm") }
    if ($leftY -lt 0.0) { $items.Add("$Name point $Index left active arm below base by $([Math]::Round(-$leftY, 3)) mm") }
    if ($rightY -lt 0.0) { $items.Add("$Name point $Index right active arm below base by $([Math]::Round(-$rightY, 3)) mm") }
    return $items
}

function Assert-PathSafe {
    param([string]$Name, [object[]]$Points)
    $count = 0
    $shown = 0
    for ($i = 0; $i -lt $Points.Count; $i++) {
        $v = Get-Violations -X ([double]$Points[$i].X) -Y ([double]$Points[$i].Y) -Index ($i + 1) -Name $Name
        foreach ($item in $v) {
            $count++
            if ($shown -lt 5) {
                Write-Host "LIMIT $item"
                $shown++
            }
        }
    }
    if ($count -gt 0) {
        throw "$Name path limit check failed with $count violations"
    }
    Write-Host ("PATH SAFE [{0}] points={1}" -f $Name, $Points.Count)
}

function New-Point {
    param([double]$X, [double]$Y)
    return [pscustomobject]@{ X = $X; Y = $Y }
}

function New-LinePoints {
    param([double]$X0, [double]$Y0, [double]$X1, [double]$Y1, [int]$N)
    $pts = @()
    for ($i = 1; $i -le $N; $i++) {
        $t = [double]$i / [Math]::Max(1, $N)
        $pts += New-Point -X ($X0 + ($X1 - $X0) * $t) -Y ($Y0 + ($Y1 - $Y0) * $t)
    }
    return $pts
}

function New-ArcPoints {
    param([double]$X0, [double]$Y0, [double]$X1, [double]$Y1, [double]$Radius, [bool]$Clockwise, [int]$N)
    $dx = $X1 - $X0
    $dy = $Y1 - $Y0
    $chord = [Math]::Sqrt($dx * $dx + $dy * $dy)
    if ($chord -lt 0.001) { throw "Arc start and target overlap" }
    $half = $chord * 0.5
    if ($Radius -lt $half) { throw "Arc radius too small: radius=$Radius min=$half" }
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
        if ($Clockwise) {
            $delta = -(($a0 - $a1) % (2.0 * [Math]::PI))
        } else {
            $delta = (($a1 - $a0) % (2.0 * [Math]::PI))
        }
        if ([Math]::Abs($delta) -lt 1e-9) {
            $delta = $(if ($Clockwise) { -2.0 * [Math]::PI } else { 2.0 * [Math]::PI })
        }
        $score = [Math]::Abs($delta)
        if ($null -eq $best -or $score -lt $best.Score) {
            $best = [pscustomobject]@{ Score = $score; Center = $c; A0 = $a0; Delta = $delta }
        }
    }

    $pts = @()
    for ($i = 1; $i -le $N; $i++) {
        $t = [double]$i / [Math]::Max(1, $N)
        $a = $best.A0 + $best.Delta * $t
        $pts += New-Point -X ($best.Center.X + $Radius * [Math]::Cos($a)) -Y ($best.Center.Y + $Radius * [Math]::Sin($a))
    }
    return $pts
}

function Get-LineLength {
    param([double]$X0, [double]$Y0, [double]$X1, [double]$Y1)
    $dx = $X1 - $X0
    $dy = $Y1 - $Y0
    return [Math]::Sqrt($dx * $dx + $dy * $dy)
}

function New-CarSegmentPoints {
    param([object[]]$Segments, [int]$N)
    $totalLen = 0.0
    foreach ($s in $Segments) { $totalLen += [double]$s.Length }
    $pts = @()
    $remaining = [Math]::Max(1, $N)
    for ($i = 0; $i -lt $Segments.Count; $i++) {
        $s = $Segments[$i]
        if ($i -eq ($Segments.Count - 1)) {
            $count = $remaining
        } else {
            $count = [Math]::Max(2, [int][Math]::Round($N * ([double]$s.Length) / $totalLen))
            $count = [Math]::Min($count, [Math]::Max(1, $remaining - (($Segments.Count - $i - 1) * 2)))
        }
        $remaining -= $count
        if ($s.Kind -eq "arc") {
            $pts += New-ArcPoints -X0 $s.X0 -Y0 $s.Y0 -X1 $s.X1 -Y1 $s.Y1 -Radius $s.Radius -Clockwise $s.Clockwise -N $count
        } else {
            $pts += New-LinePoints -X0 $s.X0 -Y0 $s.Y0 -X1 $s.X1 -Y1 $s.Y1 -N $count
        }
    }
    return $pts
}

function New-LineSeg {
    param([double]$X0, [double]$Y0, [double]$X1, [double]$Y1)
    return [pscustomobject]@{
        Kind = "line"; X0 = $X0; Y0 = $Y0; X1 = $X1; Y1 = $Y1
        Radius = 0.0; Clockwise = $false; Length = (Get-LineLength -X0 $X0 -Y0 $Y0 -X1 $X1 -Y1 $Y1)
    }
}

function New-ArcSeg {
    param([double]$X0, [double]$Y0, [double]$X1, [double]$Y1, [double]$Radius, [bool]$Clockwise)
    return [pscustomobject]@{
        Kind = "arc"; X0 = $X0; Y0 = $Y0; X1 = $X1; Y1 = $Y1
        Radius = $Radius; Clockwise = $Clockwise; Length = ([Math]::PI * $Radius)
    }
}

function New-Car1Points {
    param([double]$X0, [double]$Y0, [int]$N)
    $segments = @(
        (New-LineSeg $X0 $Y0 $X0 ($Y0 + 24.0)),
        (New-LineSeg $X0 ($Y0 + 24.0) ($X0 + 72.0) ($Y0 + 24.0)),
        (New-LineSeg ($X0 + 72.0) ($Y0 + 24.0) ($X0 + 72.0) ($Y0 + 48.0)),
        (New-LineSeg ($X0 + 72.0) ($Y0 + 48.0) ($X0 + 108.0) ($Y0 + 48.0)),
        (New-LineSeg ($X0 + 108.0) ($Y0 + 48.0) ($X0 + 120.0) ($Y0 + 36.0)),
        (New-LineSeg ($X0 + 120.0) ($Y0 + 36.0) ($X0 + 120.0) $Y0),
        (New-LineSeg ($X0 + 120.0) $Y0 ($X0 + 108.0) $Y0),
        (New-ArcSeg ($X0 + 108.0) $Y0 ($X0 + 84.0) $Y0 12.0 $true),
        (New-LineSeg ($X0 + 84.0) $Y0 ($X0 + 48.0) $Y0),
        (New-ArcSeg ($X0 + 48.0) $Y0 ($X0 + 24.0) $Y0 12.0 $true),
        (New-LineSeg ($X0 + 24.0) $Y0 $X0 $Y0)
    )
    return New-CarSegmentPoints -Segments $segments -N $N
}

function New-Car2Points {
    param([double]$X0, [double]$Y0, [int]$N)
    $segments = @(
        (New-LineSeg $X0 $Y0 $X0 ($Y0 + 20.0)),
        (New-LineSeg $X0 ($Y0 + 20.0) ($X0 + 40.0) ($Y0 + 20.0)),
        (New-LineSeg ($X0 + 40.0) ($Y0 + 20.0) ($X0 + 60.0) ($Y0 + 40.0)),
        (New-LineSeg ($X0 + 60.0) ($Y0 + 40.0) ($X0 + 120.0) ($Y0 + 40.0)),
        (New-LineSeg ($X0 + 120.0) ($Y0 + 40.0) ($X0 + 140.0) ($Y0 + 20.0)),
        (New-LineSeg ($X0 + 140.0) ($Y0 + 20.0) ($X0 + 160.0) ($Y0 + 20.0)),
        (New-LineSeg ($X0 + 160.0) ($Y0 + 20.0) ($X0 + 160.0) $Y0),
        (New-LineSeg ($X0 + 160.0) $Y0 ($X0 + 140.0) $Y0),
        (New-ArcSeg ($X0 + 140.0) $Y0 ($X0 + 116.0) $Y0 12.0 $true),
        (New-LineSeg ($X0 + 116.0) $Y0 ($X0 + 44.0) $Y0),
        (New-ArcSeg ($X0 + 44.0) $Y0 ($X0 + 20.0) $Y0 12.0 $true),
        (New-LineSeg ($X0 + 20.0) $Y0 $X0 $Y0)
    )
    return New-CarSegmentPoints -Segments $segments -N $N
}

function New-G1Line {
    param([double]$UiX, [double]$UiY, [int]$Id)
    $mcuX = Convert-UiToMcuX -X $UiX
    return ("G1 X{0:F3} Y{1:F3} F{2:F0} ;ID=TRJ{3:D4} LIM=1" -f $mcuX, $UiY, $FeedMmMin, $Id)
}

function Send-Expect {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string]$Line,
        [string[]]$AcceptPrefixes,
        [string]$Name,
        [switch]$RequireEcho
    )
    $expectedCs = Get-Checksum -Line $Line
    $Serial.WriteLine($Line)
    $deadline = (Get-Date).AddMilliseconds($CommandTimeoutMs)
    while ($true) {
        if ((Get-Date) -gt $deadline) { throw "Timeout waiting response for $Name" }
        $rx = Read-Line-Safe -Serial $Serial
        if ($rx.StartsWith("<")) { continue }
        if ($rx.StartsWith("error:") -or $rx.StartsWith("ERR ") -or $rx -eq "<timeout>") {
            throw "$Name returned $rx"
        }
        foreach ($prefix in $AcceptPrefixes) {
            if ($rx.StartsWith($prefix)) {
                if ($RequireEcho) {
                    if ($rx -notmatch "cs=$expectedCs") { throw "$Name ACK checksum mismatch expected $expectedCs" }
                    if (-not $rx.EndsWith("line=$Line")) { throw "$Name ACK line mismatch" }
                }
                return
            }
        }
    }
}

function Wait-Idle {
    param([System.IO.Ports.SerialPort]$Serial, [string]$Name)
    $deadline = (Get-Date).AddMilliseconds($DrainTimeoutMs)
    while ((Get-Date) -lt $deadline) {
        $Serial.Write("?")
        $rx = Read-Line-Safe -Serial $Serial
        Write-Host ("RX [drain {0}] {1}" -f $Name, $rx)
        if ($rx.StartsWith("<Idle") -and $rx.Contains("Q:0") -and $rx.Contains("E:0")) {
            return
        }
        Start-Sleep -Milliseconds 200
    }
    throw "$Name did not drain to Idle/Q:0/E:0"
}

function Send-Path {
    param([System.IO.Ports.SerialPort]$Serial, [string]$Name, [object[]]$Points, [int]$StartId)
    Assert-PathSafe -Name $Name -Points $Points
    $sent = 0
    for ($i = 0; $i -lt $Points.Count; $i++) {
        $line = New-G1Line -UiX ([double]$Points[$i].X) -UiY ([double]$Points[$i].Y) -Id ($StartId + $i)
        Send-Expect -Serial $Serial -Line $line -AcceptPrefixes @("ok") -Name "$Name $($i + 1)/$($Points.Count)" -RequireEcho
        $sent++
        if (($sent % 100) -eq 0) {
            Write-Host ("PROGRESS [{0}] {1}/{2}" -f $Name, $sent, $Points.Count)
        }
    }
    Wait-Idle -Serial $Serial -Name $Name
    Write-Host ("PATH PASS [{0}] sent={1}" -f $Name, $sent)
    return $sent
}

$Port = Resolve-SerialPortName -Requested $Port
$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = $TimeoutMs
$serial.WriteTimeout = $TimeoutMs

try {
    $perPath = [Math]::Max(20, [int][Math]::Floor($Count / 5))
    Write-Host "Opening $Port at $Baud 8N1 ..."
    Write-Host ("UI trajectory stress: line={0}, G2={0}, G3={0}, car1={0}, car2={0}, feed={1:F0}" -f $perPath, $FeedMmMin)
    $serial.Open()
    Start-Sleep -Milliseconds 300
    $serial.DiscardInBuffer()

    Send-Expect -Serial $serial -Line "VERSION" -AcceptPrefixes @("OK VERSION") -Name "VERSION"
    Send-Expect -Serial $serial -Line "HOSTCAP" -AcceptPrefixes @("OK HOSTCAP") -Name "HOSTCAP"
    Send-Expect -Serial $serial -Line "WATCHDOG OFF" -AcceptPrefixes @("OK WATCHDOG") -Name "WATCHDOG OFF"
    Send-Expect -Serial $serial -Line "CLEAR_ERROR" -AcceptPrefixes @("OK CLEAR_ERROR") -Name "CLEAR_ERROR"
    Send-Expect -Serial $serial -Line "ENABLE 1" -AcceptPrefixes @("OK ENABLE 1") -Name "ENABLE 1"
    Send-Expect -Serial $serial -Line "ZERO" -AcceptPrefixes @("OK ZERO") -Name "ZERO"
    Send-Expect -Serial $serial -Line "G21" -AcceptPrefixes @("ok") -Name "G21" -RequireEcho
    Send-Expect -Serial $serial -Line "G90" -AcceptPrefixes @("ok") -Name "G90" -RequireEcho

    $line = New-LinePoints -X0 75.0 -Y0 220.0 -X1 110.0 -Y1 235.0 -N $perPath
    $cw = New-ArcPoints -X0 110.0 -Y0 235.0 -X1 75.0 -Y1 250.0 -Radius 45.0 -Clockwise $true -N $perPath
    $ccw = New-ArcPoints -X0 75.0 -Y0 250.0 -X1 75.0 -Y1 220.0 -Radius 35.0 -Clockwise $false -N $perPath
    $car1 = New-Car1Points -X0 75.0 -Y0 200.0 -N $perPath
    $car2 = New-Car2Points -X0 75.0 -Y0 200.0 -N $perPath

    $total = 0
    $total += Send-Path -Serial $serial -Name "G1 line" -Points $line -StartId 1
    $total += Send-Path -Serial $serial -Name "G2 clockwise arc" -Points $cw -StartId (1 + $line.Count)
    $total += Send-Path -Serial $serial -Name "G3 counter-clockwise arc" -Points $ccw -StartId (1 + $line.Count + $cw.Count)
    $total += Send-Path -Serial $serial -Name "car outline 1" -Points $car1 -StartId (1 + $line.Count + $cw.Count + $ccw.Count)
    $total += Send-Path -Serial $serial -Name "car outline 2" -Points $car2 -StartId (1 + $line.Count + $cw.Count + $ccw.Count + $car1.Count)

    if (-not $KeepEnabled) {
        Send-Expect -Serial $serial -Line "ENABLE 0" -AcceptPrefixes @("OK ENABLE 0") -Name "ENABLE 0"
    }
    Write-Host ("UI_TRAJECTORY_STRESS PASS total={0}" -f $total)
    exit 0
} catch {
    Write-Host "UI_TRAJECTORY_STRESS FAIL: $($_.Exception.Message)"
    exit 1
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
