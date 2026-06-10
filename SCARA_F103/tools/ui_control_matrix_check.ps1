param(
    [string]$Port = "AUTO",
    [int]$Baud = 115200,
    [int]$TimeoutMs = 2000,
    [int]$DrainTimeoutMs = 30000,
    [double]$FeedMmMin = 900.0,
    [switch]$SkipLongPath
)

# 上位机控键协议矩阵检查：
# 按 SCARA_UI 当前控键语义生成等价串口指令，逐项验证 ACK、状态、错误原因。

$ErrorActionPreference = "Stop"

$BaseMm = 150.0
$ActiveMm = 160.0
$PassiveMm = 200.0
$HomeX = 75.0
$HomeY = 345.3
$SafeControlX = 75.0
$SafeControlY = 220.0
$JogMm = 10.0
$MotorJogDeg = 3.0

function Resolve-SerialPortName {
    param([string]$Requested)
    if ($Requested -and $Requested.ToUpperInvariant() -ne "AUTO") {
        return $Requested
    }

    $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    if ($ports.Count -eq 1) {
        Write-Host ("AUTO port selected: {0}" -f $ports[0])
        return $ports[0]
    }
    if ($ports.Count -eq 0) {
        throw "未发现串口，请检查 USB 转串口/单片机连接，或使用 -Port COMx 指定端口。"
    }
    throw ("发现多个串口：{0}；请使用 -Port COMx 明确指定。" -f ($ports -join ", "))
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

function Test-Reachable {
    param([double]$X, [double]$Y)
    $d1 = [Math]::Sqrt($X * $X + $Y * $Y)
    $d2 = [Math]::Sqrt(($X - $BaseMm) * ($X - $BaseMm) + $Y * $Y)
    if ($Y -lt 15.0) { return $false }
    if ($d1 -gt 355.0 -or $d2 -gt 355.0 -or $d1 -lt 45.0 -or $d2 -lt 45.0) { return $false }
    return $true
}

function Get-InverseDeg {
    param([double]$X, [double]$Y)
    $d1 = [Math]::Sqrt($X * $X + $Y * $Y)
    $d2 = [Math]::Sqrt(($X - $BaseMm) * ($X - $BaseMm) + $Y * $Y)
    if ($d1 -le 0.001 -or $d2 -le 0.001) { throw "逆解失败：距离过小" }
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

function Get-ForwardXY {
    param([double]$Q1Deg, [double]$Q2Deg)
    $q1 = $Q1Deg * [Math]::PI / 180.0
    $q2 = $Q2Deg * [Math]::PI / 180.0
    $c1x = $ActiveMm * [Math]::Cos($q1)
    $c1y = $ActiveMm * [Math]::Sin($q1)
    $c2x = $BaseMm + $ActiveMm * [Math]::Cos($q2)
    $c2y = $ActiveMm * [Math]::Sin($q2)
    $dx = $c2x - $c1x
    $dy = $c2y - $c1y
    $d = [Math]::Sqrt($dx * $dx + $dy * $dy)
    if ($d -le 0.001 -or $d -gt 2.0 * $PassiveMm) { throw "正解失败：连杆交点不可达" }
    $mx = ($c1x + $c2x) * 0.5
    $my = ($c1y + $c2y) * 0.5
    $h = [Math]::Sqrt([Math]::Max(0.0, ($PassiveMm * $PassiveMm) - (($d * 0.5) * ($d * 0.5))))
    $rx = -$dy / $d
    $ry = $dx / $d
    [double]$x1 = $mx + ($h * $rx)
    [double]$y1 = $my + ($h * $ry)
    [double]$x2 = $mx - ($h * $rx)
    [double]$y2 = $my - ($h * $ry)
    if ($y1 -ge $y2) {
        return [pscustomobject]@{ X = $x1; Y = $y1 }
    }
    return [pscustomobject]@{ X = $x2; Y = $y2 }
}

function New-G1Line {
    param([double]$UiX, [double]$UiY, [int]$Id)
    $mcuX = Convert-UiToMcuX -X $UiX
    return ("G1 X{0:F3} Y{1:F3} F{2:F0} ;ID=UI{3:D3} LIM=1" -f $mcuX, $UiY, $FeedMmMin, $Id)
}

function Explain-Error {
    param([string]$Rx)
    if ($Rx.StartsWith("error:8")) {
        return "发送太快或下位机 pending/buffer 忙；应等待 ok 或等待状态 Bf/Q 恢复。"
    }
    if ($Rx.StartsWith("error:15")) {
        return "运动被拒绝；常见原因是逆解失败、电机未使能、急停或底层错误位未清除。"
    }
    if ($Rx.StartsWith("error:4")) {
        return "F 速度字段非法。"
    }
    if ($Rx.StartsWith("error:20")) {
        return "G-code 字段不支持。"
    }
    return "未知错误，需要结合 ERRORS 和状态帧继续判断。"
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
    Write-Host ("TX [{0}] {1}" -f $Name, $Line)
    $Serial.WriteLine($Line)
    while ($true) {
        $rx = Read-Line-Safe -Serial $Serial
        Write-Host ("RX [{0}] {1}" -f $Name, $rx)
        if ($rx.StartsWith("<")) { continue }
        if ($rx.StartsWith("error:") -or $rx.StartsWith("ERR ") -or $rx -eq "<timeout>") {
            Write-Host ("DIAG [{0}] {1}" -f $Name, (Explain-Error -Rx $rx))
            if ($rx.StartsWith("error:15")) {
                $Serial.WriteLine("ERRORS")
                $Serial.Write("?")
            }
            throw "控键 $Name 返回错误：$rx"
        }
        foreach ($prefix in $AcceptPrefixes) {
            if ($rx.StartsWith($prefix)) {
                if ($RequireEcho) {
                    if ($rx -notmatch "cs=$expectedCs") { throw "控键 $Name ACK 校验和不匹配，期望 $expectedCs" }
                    if (-not $rx.EndsWith("line=$Line")) { throw "控键 $Name ACK line 回显不匹配" }
                }
                Write-Host ("PASS [{0}]" -f $Name)
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
        Start-Sleep -Milliseconds 150
    }
    throw "控键 $Name 后队列未在 $DrainTimeoutMs ms 内回到 Idle/Q:0/E:0"
}

function Send-UiMove {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string]$Name,
        [double]$UiX,
        [double]$UiY,
        [int]$Id
    )
    if (-not (Test-Reachable -X $UiX -Y $UiY)) {
        throw "控键 $Name 的目标点超出上位机工作空间：X=$UiX Y=$UiY"
    }
    $line = New-G1Line -UiX $UiX -UiY $UiY -Id $Id
    Send-Expect -Serial $Serial -Line $line -AcceptPrefixes @("ok") -Name $Name -RequireEcho
    Wait-Idle -Serial $Serial -Name $Name
}

$Port = Resolve-SerialPortName -Requested $Port
$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = $TimeoutMs
$serial.WriteTimeout = $TimeoutMs

try {
    Write-Host "Opening $Port at $Baud 8N1 ..."
    $serial.Open()
    Start-Sleep -Milliseconds 300
    $serial.DiscardInBuffer()

    Send-Expect -Serial $serial -Line "VERSION" -AcceptPrefixes @("OK VERSION") -Name "连接/VERSION"
    Send-Expect -Serial $serial -Line "HOSTCAP" -AcceptPrefixes @("OK HOSTCAP") -Name "连接/HOSTCAP"
    Send-Expect -Serial $serial -Line "WATCHDOG OFF" -AcceptPrefixes @("OK WATCHDOG") -Name "通信看门狗关闭"
    Send-Expect -Serial $serial -Line "CLEAR_ERROR" -AcceptPrefixes @("OK CLEAR_ERROR") -Name "清错"
    Send-Expect -Serial $serial -Line "ENABLE 1" -AcceptPrefixes @("OK ENABLE 1") -Name "使能"
    Send-Expect -Serial $serial -Line "ZERO" -AcceptPrefixes @("OK ZERO") -Name "系统一键复位/软件零点"

    Send-UiMove -Serial $serial -Name "启动位到安全操作点" -UiX $SafeControlX -UiY $SafeControlY -Id 0
    $x = $SafeControlX
    $y = $SafeControlY
    Send-UiMove -Serial $serial -Name "前进点动" -UiX $x -UiY ($y + $JogMm) -Id 1
    $y += $JogMm
    Send-UiMove -Serial $serial -Name "后退点动" -UiX $x -UiY ($y - $JogMm) -Id 2
    $y -= $JogMm
    Send-UiMove -Serial $serial -Name "左移点动" -UiX ($x - $JogMm) -UiY $y -Id 3
    $x -= $JogMm
    Send-UiMove -Serial $serial -Name "右移点动" -UiX ($x + $JogMm) -UiY $y -Id 4
    $x += $JogMm

    $q = Get-InverseDeg -X $x -Y $y
    $q1 = [double]$q.Q1
    $q2 = [double]$q.Q2
    $p = Get-ForwardXY -Q1Deg ($q1 + $MotorJogDeg) -Q2Deg $q2
    Send-UiMove -Serial $serial -Name "M1+ 电机点动" -UiX $p.X -UiY $p.Y -Id 5
    $x = $p.X; $y = $p.Y
    $q = Get-InverseDeg -X $x -Y $y
    $q1 = [double]$q.Q1
    $q2 = [double]$q.Q2
    $p = Get-ForwardXY -Q1Deg ($q1 - $MotorJogDeg) -Q2Deg $q2
    Send-UiMove -Serial $serial -Name "M1- 电机点动" -UiX $p.X -UiY $p.Y -Id 6
    $x = $p.X; $y = $p.Y
    $q = Get-InverseDeg -X $x -Y $y
    $q1 = [double]$q.Q1
    $q2 = [double]$q.Q2
    $p = Get-ForwardXY -Q1Deg $q1 -Q2Deg ($q2 + $MotorJogDeg)
    Send-UiMove -Serial $serial -Name "M2+ 电机点动" -UiX $p.X -UiY $p.Y -Id 7
    $x = $p.X; $y = $p.Y
    $q = Get-InverseDeg -X $x -Y $y
    $q1 = [double]$q.Q1
    $q2 = [double]$q.Q2
    $p = Get-ForwardXY -Q1Deg $q1 -Q2Deg ($q2 - $MotorJogDeg)
    Send-UiMove -Serial $serial -Name "M2- 电机点动" -UiX $p.X -UiY $p.Y -Id 8
    $x = $p.X; $y = $p.Y

    Send-UiMove -Serial $serial -Name "轨迹规划按钮/默认目标" -UiX 150.0 -UiY 250.0 -Id 9
    $x = 150.0; $y = 250.0

    if (-not $SkipLongPath) {
        Send-UiMove -Serial $serial -Name "小车轨迹/起点" -UiX 75.0 -UiY 200.0 -Id 10
        Send-UiMove -Serial $serial -Name "小车轨迹/终点" -UiX 235.0 -UiY 200.0 -Id 11
    } else {
        Write-Host "SKIP 小车长路径：已指定 -SkipLongPath"
    }

    Write-Host "INFO 示教记录/结束/清除、颜色识别、边缘检测、清空绘图、清空日志为上位机本地状态操作，不直接发送运动协议。"
    Write-Host "INFO 摄像头与视觉循迹需要真实画面；无画面时 UI 正确行为是不发送运动并提示“无画面”。"

    Send-Expect -Serial $serial -Line "ESTOP" -AcceptPrefixes @("OK ESTOP") -Name "紧急停止"
    Send-Expect -Serial $serial -Line "CLEAR_ERROR" -AcceptPrefixes @("OK CLEAR_ERROR") -Name "急停后清错"
    Send-Expect -Serial $serial -Line "ENABLE 0" -AcceptPrefixes @("OK ENABLE 0") -Name "测试结束释放电机"
    Write-Host "UI CONTROL MATRIX PASS"
    exit 0
} catch {
    Write-Host "UI CONTROL MATRIX FAIL: $($_.Exception.Message)"
    exit 1
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
