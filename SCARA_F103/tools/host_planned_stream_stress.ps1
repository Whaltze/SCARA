param(
    [string]$Port = "COM13",
    [int]$Baud = 115200,
    [int]$TimeoutMs = 5000,
    [int]$CommandTimeoutMs = 60000,
    [int]$DrainTimeoutMs = 120000,
    [int]$Count = 3000,
    [int]$FeedMin = 500,
    [int]$FeedMax = 1800,
    [double]$MinX = -20.0,
    [double]$MaxX = 20.0,
    [double]$MinY = 90.0,
    [double]$MaxY = 150.0,
    [switch]$EnableMotion,
    [switch]$KeepEnabled,
    [switch]$VerboseLines,
    [switch]$QuietLines
)

# 课程设计主压测脚本：模拟上位机生成轨迹、规划速度、判断限位，再逐行发送 G-code。
# 默认显示每条 TX/RX/MATCH，方便观察协议格式和每条指令是否匹配。

$ErrorActionPreference = "Stop"

function Get-LineChecksum {
    param([string]$Line)
    $sum = 0
    foreach ($b in [System.Text.Encoding]::ASCII.GetBytes($Line)) {
        $sum = ($sum + $b) -band 0xFF
    }
    return "{0:X2}" -f $sum
}

function Read-Line-Safe {
    param([System.IO.Ports.SerialPort]$Serial)
    try { return $Serial.ReadLine().Trim() } catch [TimeoutException] { return "<timeout>" }
}

function Send-Command {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string]$Line,
        [switch]$RequireEcho,
        [string[]]$AcceptPrefixes = @("ok"),
        [string]$Name = $Line
    )

    $expectedCs = Get-LineChecksum -Line $Line
    $showLine = $VerboseLines -or -not $QuietLines
    $txTime = Get-Date -Format "HH:mm:ss.fff"
    if ($showLine) {
        Write-Host ("TX {0} [{1}] cs={2} len={3} line={4}" -f $txTime, $Name, $expectedCs, $Line.Length, $Line)
    }

    $Serial.WriteLine($Line)
    $deadline = (Get-Date).AddMilliseconds($CommandTimeoutMs)
    while ($true) {
        if ((Get-Date) -gt $deadline) { throw "Timeout waiting response for $Name" }
        $rx = Read-Line-Safe -Serial $Serial
        $rxTime = Get-Date -Format "HH:mm:ss.fff"

        if ($showLine -or $rx.StartsWith("error:") -or $rx.StartsWith("ERR ") -or $rx -eq "<timeout>") {
            $kind = if ($rx.StartsWith("<")) { "RX STATUS" } else { "RX" }
            Write-Host ("{0} {1} [{2}] {3}" -f $kind, $rxTime, $Name, $rx)
        }

        if ($rx.StartsWith("<")) { continue }
        foreach ($prefix in $AcceptPrefixes) {
            if ($rx.StartsWith($prefix)) {
                if ($RequireEcho -and $rx.Trim().ToLowerInvariant() -ne "ok") {
                    if ($rx -notmatch "cs=$expectedCs") { throw "ACK checksum mismatch for $Name expected cs=$expectedCs" }
                    if (-not $rx.EndsWith("line=$Line")) { throw "ACK line mismatch for $Name" }
                }
                if ($showLine) { Write-Host ("MATCH [{0}] ack={1} cs={2}" -f $Name, $rx, $expectedCs) }
                return $rx
            }
        }

        if ($rx.StartsWith("error:") -or $rx.StartsWith("ERR ") -or $rx -eq "<timeout>") {
            throw "Unexpected response for $Name : $rx"
        }
    }
}

function Assert-HostLimits {
    param([double]$X, [double]$Y, [int]$Feed)

    if ($X -lt $MinX -or $X -gt $MaxX -or $Y -lt $MinY -or $Y -gt $MaxY) {
        throw ("Host limit reject X={0:F3} Y={1:F3}" -f $X, $Y)
    }
    if ($Feed -lt $FeedMin -or $Feed -gt $FeedMax) {
        throw "Host feed reject F=$Feed"
    }
}

function Get-HostPlannedPoint {
    param([int]$PointIndex, [int]$Total)

    $u = [double]$PointIndex / [Math]::Max(1, $Total - 1)
    if ($u -lt 0.34) {
        $local = $u / 0.34
        $x = -8.0 + 16.0 * $local
        $y = 118.0 + 4.0 * [Math]::Sin([Math]::PI * $local)
    } elseif ($u -lt 0.67) {
        $local = ($u - 0.34) / 0.33
        $theta = 2.0 * [Math]::PI * $local
        $x = 8.0 + 3.0 * [Math]::Cos($theta)
        $y = 122.0 + 3.0 * [Math]::Sin($theta)
    } else {
        $local = ($u - 0.67) / 0.33
        $x = 8.0 - 16.0 * $local
        $y = 122.0 - 4.0 * [Math]::Sin([Math]::PI * $local)
    }

    $speedWave = 0.5 + 0.5 * [Math]::Sin(2.0 * [Math]::PI * 4.0 * $u)
    $feed = [int][Math]::Round($FeedMin + ($FeedMax - $FeedMin) * $speedWave)
    Assert-HostLimits -X $x -Y $y -Feed $feed

    $id = "{0:D4}" -f ($PointIndex + 1)
    return ("G1 X{0:F3} Y{1:F3} F{2} ;ID={3} LIM=1" -f $x, $y, $feed, $id)
}

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = $TimeoutMs
$serial.WriteTimeout = $TimeoutMs
$started = Get-Date
$okCount = 0

try {
    Write-Host "Opening $Port at $Baud 8N1 ..."
    Write-Host ("Host-planned stream: {0} points, host feed {1}-{2}, host limits X[{3},{4}] Y[{5},{6}]" -f $Count, $FeedMin, $FeedMax, $MinX, $MaxX, $MinY, $MaxY)
    Write-Host "Protocol line example: G1 X-8.000 Y118.000 F1150 ;ID=0001 LIM=1"
    Write-Host "Expected ACK: ok (legacy ok seq/cs/line is still accepted)"
    if ($QuietLines) {
        Write-Host "Line display: quiet. Remove -QuietLines to print every TX/RX pair."
    } else {
        Write-Host "Line display: verbose by default. Use -QuietLines to show only progress/errors."
    }
    if (-not $EnableMotion) {
        Write-Host "WARNING: -EnableMotion was not provided. The test validates receive/ACK first; planner drain may need enabled motors."
    }

    $serial.Open()
    Start-Sleep -Milliseconds 300
    $serial.DiscardInBuffer()

    Send-Command -Serial $serial -Line "VERSION" -AcceptPrefixes @("OK VERSION") -Name "VERSION" | Out-Null
    Send-Command -Serial $serial -Line "HOSTCAP" -AcceptPrefixes @("OK HOSTCAP") -Name "HOSTCAP" | Out-Null
    Send-Command -Serial $serial -Line "WATCHDOG OFF" -AcceptPrefixes @("OK WATCHDOG") -Name "WATCHDOG OFF" | Out-Null
    Send-Command -Serial $serial -Line "CLEAR_ERROR" -AcceptPrefixes @("OK CLEAR_ERROR") -Name "CLEAR_ERROR" | Out-Null
    if ($EnableMotion) {
        Send-Command -Serial $serial -Line "ENABLE 1" -AcceptPrefixes @("OK ENABLE 1") -Name "ENABLE 1" | Out-Null
    }

    Send-Command -Serial $serial -Line "G21" -RequireEcho -Name "G21" | Out-Null
    Send-Command -Serial $serial -Line "G90" -RequireEcho -Name "G90" | Out-Null
    Send-Command -Serial $serial -Line "G0 X-8.000 Y118.000 ;ID=SEED LIM=1" -RequireEcho -Name "seed move" | Out-Null

    for ($i = 0; $i -lt $Count; $i++) {
        $line = Get-HostPlannedPoint -PointIndex $i -Total $Count
        Send-Command -Serial $serial -Line $line -RequireEcho -Name "point $($i + 1)/$Count" | Out-Null
        $okCount++
        if (($okCount % 100) -eq 0) {
            $elapsed = ((Get-Date) - $started).TotalSeconds
            Write-Host ("PROGRESS {0}/{1} elapsed={2:F1}s" -f $okCount, $Count, $elapsed)
        }
    }

    if ($EnableMotion) {
        $drainDeadline = (Get-Date).AddMilliseconds($DrainTimeoutMs)
        while ($true) {
            if ((Get-Date) -gt $drainDeadline) { throw "Planner did not drain before $DrainTimeoutMs ms" }
            $serial.Write("?")
            $rx = Read-Line-Safe -Serial $serial
            Write-Host "RX $rx"
            if ($rx.StartsWith("<Idle") -and $rx.Contains("Q:0")) { break }
            if ($rx -eq "<timeout>") { throw "No final status" }
            Start-Sleep -Milliseconds 200
        }
    } else {
        $serial.Write("?")
        while ($true) {
            $rx = Read-Line-Safe -Serial $serial
            Write-Host "RX $rx"
            if ($rx.StartsWith("<")) { break }
            if ($rx -eq "<timeout>") { throw "No status" }
        }
    }

    if ($EnableMotion -and -not $KeepEnabled) {
        Send-Command -Serial $serial -Line "ENABLE 0" -AcceptPrefixes @("OK ENABLE 0") -Name "ENABLE 0" | Out-Null
    }

    $elapsedTotal = ((Get-Date) - $started).TotalSeconds
    Write-Host ("HOST_PLANNED PASS ok={0} elapsed={1:F1}s" -f $okCount, $elapsedTotal)
    exit 0
} catch {
    Write-Host "HOST_PLANNED FAIL: $($_.Exception.Message)"
    exit 1
} finally {
    if ($serial.IsOpen) { $serial.Close() }
}
