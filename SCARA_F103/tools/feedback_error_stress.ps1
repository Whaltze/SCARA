param(
    [string]$Port = "COM13",
    [int]$Baud = 115200,
    [int]$TimeoutMs = 1000,
    [int]$CommandTimeoutMs = 20000,
    [int]$DrainTimeoutMs = 120000,
    [int]$Count = 1000,
    [double]$FeedMmMin = 900.0,
    [double]$MaxErrorMm = 1.0,
    [string]$CsvPath = "",
    [switch]$EnableMotion,
    [switch]$KeepEnabled
)

$ErrorActionPreference = "Stop"

$expected = New-Object System.Collections.Generic.List[object]
$samples = New-Object System.Collections.Generic.List[object]

function Get-LineChecksum {
    param([string]$Line)
    $sum = 0
    foreach ($b in [System.Text.Encoding]::ASCII.GetBytes($Line)) {
        $sum = ($sum + $b) -band 0xFF
    }
    return "{0:X2}" -f $sum
}

function Add-ExpectedPoint {
    param([double]$X, [double]$Y)
    $script:expected.Add([pscustomobject]@{ X = $X; Y = $Y })
}

function Get-ExpectedPoint {
    param([int]$Index, [int]$Total)
    $u = [double]$Index / [Math]::Max(1, $Total - 1)
    $x = -20.0 + 40.0 * $u
    $y = 145.0 + 18.0 * [Math]::Sin(2.0 * [Math]::PI * $u)
    return [pscustomobject]@{ X = $x; Y = $y }
}

function Project-To-Segment {
    param(
        [double]$Px, [double]$Py,
        [double]$Ax, [double]$Ay,
        [double]$Bx, [double]$By
    )
    $vx = $Bx - $Ax
    $vy = $By - $Ay
    $den = $vx * $vx + $vy * $vy
    if ($den -le 1e-12) {
        return [pscustomobject]@{ X = $Ax; Y = $Ay }
    }
    $t = (($Px - $Ax) * $vx + ($Py - $Ay) * $vy) / $den
    if ($t -lt 0.0) { $t = 0.0 }
    if ($t -gt 1.0) { $t = 1.0 }
    return [pscustomobject]@{ X = $Ax + $vx * $t; Y = $Ay + $vy * $t }
}

function Add-FeedbackSample {
    param([double]$X, [double]$Y)
    if ($script:expected.Count -eq 0) { return }
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

function Parse-Status {
    param([string]$Line)
    if ($Line -match "MPos:([-0-9.]+),([-0-9.]+)") {
        Add-FeedbackSample -X ([double]$Matches[1]) -Y ([double]$Matches[2])
    }
}

function Read-Line-Safe {
    param([System.IO.Ports.SerialPort]$Serial)
    try { return $Serial.ReadLine().Trim() } catch [TimeoutException] { return "<timeout>" }
}

function Send-Command {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string]$Line,
        [string[]]$AcceptPrefixes = @("ok"),
        [switch]$RequireEcho,
        [string]$Name = $Line
    )

    $expectedCs = Get-LineChecksum -Line $Line
    $Serial.WriteLine($Line)
    $deadline = (Get-Date).AddMilliseconds($CommandTimeoutMs)
    while ($true) {
        if ((Get-Date) -gt $deadline) { throw "Timeout waiting response for $Name" }
        $rx = Read-Line-Safe -Serial $Serial
        if ($rx.StartsWith("<")) {
            Parse-Status -Line $rx
            continue
        }
        foreach ($prefix in $AcceptPrefixes) {
            if ($rx.StartsWith($prefix)) {
                if ($RequireEcho -and $rx.Trim().ToLowerInvariant() -ne "ok") {
                    if ($rx -notmatch "cs=$expectedCs") { throw "ACK checksum mismatch for $Name" }
                    if (-not $rx.EndsWith("line=$Line")) { throw "ACK line mismatch for $Name" }
                }
                return $rx
            }
        }
        if ($rx.StartsWith("error:") -or $rx.StartsWith("ERR ") -or $rx -eq "<timeout>") {
            throw "Unexpected response for $Name : $rx"
        }
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

for ($i = 0; $i -lt $Count; $i++) {
    $p = Get-ExpectedPoint -Index $i -Total $Count
    Add-ExpectedPoint -X $p.X -Y $p.Y
}

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = $TimeoutMs
$serial.WriteTimeout = $TimeoutMs

try {
    Write-Host "Opening $Port at $Baud 8N1 ..."
    Write-Host ("Feedback error stress: points={0} feed={1:F0} max_error={2:F3}mm csv={3}" -f $Count, $FeedMmMin, $MaxErrorMm, $(if ($CsvPath) { $CsvPath } else { "-" }))
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
    $first = $expected[0]
    Send-Command -Serial $serial -Line ("G0 X{0:F3} Y{1:F3} ;ID=SEED LIM=1" -f $first.X, $first.Y) -RequireEcho -Name "seed" | Out-Null

    for ($i = 0; $i -lt $expected.Count; $i++) {
        $p = $expected[$i]
        $line = "G1 X{0:F3} Y{1:F3} F{2:F0} ;ID={3:D4} LIM=1" -f $p.X, $p.Y, $FeedMmMin, ($i + 1)
        Send-Command -Serial $serial -Line $line -RequireEcho -Name "point $($i + 1)/$Count" | Out-Null
        if ((($i + 1) % 100) -eq 0) {
            $summary = Get-ErrorSummary
            Write-Host ("PROGRESS {0}/{1} samples={2} max={3:F4}mm rms={4:F4}mm" -f ($i + 1), $Count, $summary.Count, $summary.MaxE, $summary.RmsE)
        }
    }

    if ($EnableMotion) {
        $deadline = (Get-Date).AddMilliseconds($DrainTimeoutMs)
        while ($true) {
            if ((Get-Date) -gt $deadline) { throw "Motion did not drain before timeout" }
            $serial.Write("?")
            $rx = Read-Line-Safe -Serial $serial
            if ($rx.StartsWith("<")) {
                Parse-Status -Line $rx
                if ($rx.StartsWith("<Idle") -and $rx.Contains("Q:0")) { break }
            }
            Start-Sleep -Milliseconds 100
        }
    }

    $summary = Get-ErrorSummary
    Write-Host ("FEEDBACK_ERROR samples={0} max_x={1:F4} max_y={2:F4} rms_x={3:F4} rms_y={4:F4} max_norm={5:F4} rms_norm={6:F4}" -f $summary.Count, $summary.MaxX, $summary.MaxY, $summary.RmsX, $summary.RmsY, $summary.MaxE, $summary.RmsE)
    Export-FeedbackCsv
    if ($summary.Count -eq 0) {
        throw "No feedback samples were captured"
    }
    if ($summary.MaxE -gt $MaxErrorMm) {
        throw ("Feedback error exceeded limit: max={0:F4}mm limit={1:F4}mm" -f $summary.MaxE, $MaxErrorMm)
    }

    if ($EnableMotion -and -not $KeepEnabled) {
        Send-Command -Serial $serial -Line "ENABLE 0" -AcceptPrefixes @("OK ENABLE 0") -Name "ENABLE 0" | Out-Null
    }
    Write-Host "FEEDBACK_ERROR_STRESS PASS"
    exit 0
} catch {
    Write-Host "FEEDBACK_ERROR_STRESS FAIL: $($_.Exception.Message)"
    exit 1
} finally {
    if ($serial.IsOpen) { $serial.Close() }
}
