param(
    [string]$Port = "AUTO",
    [int]$Count = 300,
    [double]$FeedMmMin = 900.0,
    [double]$MaxErrorMm = 1.0,
    [double]$MaxRmsMm = 0.3,
    [string]$OutDir = "",
    [switch]$SkipMotion
)

$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
if ($Port.ToUpperInvariant() -eq "AUTO") {
    $ports = @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)
    if ($ports.Count -ne 1) {
        throw "AUTO requires exactly one serial port; found: $($ports -join ', ')"
    }
    $Port = $ports[0]
}
if ([string]::IsNullOrWhiteSpace($OutDir)) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $OutDir = Join-Path $projectRoot "logs\final_validation_$stamp"
}
if (-not [System.IO.Path]::IsPathRooted($OutDir)) {
    $OutDir = Join-Path $projectRoot $OutDir
}
$OutDir = [System.IO.Path]::GetFullPath($OutDir)
if (-not (Test-Path -LiteralPath $OutDir)) {
    New-Item -ItemType Directory -Path $OutDir | Out-Null
}

function Invoke-Step {
    param(
        [string]$Name,
        [string]$File,
        [string[]]$StepArgs
    )
    Write-Host "== $Name =="
    & powershell -NoProfile -ExecutionPolicy Bypass -File $File @StepArgs
    if ($LASTEXITCODE -ne 0) {
        throw "$Name failed exit=$LASTEXITCODE"
    }
}

Push-Location $projectRoot
try {
    Invoke-Step -Name "Project Verify" -File (Join-Path $PSScriptRoot "verify_project.ps1") -StepArgs @()
    Invoke-Step -Name "Serial Link" -File (Join-Path $PSScriptRoot "serial_link_check.ps1") -StepArgs @(
        "-Port", $Port,
        "-Repeat", "2"
    )
    Invoke-Step -Name "Buffered Stream Capability" -File (Join-Path $PSScriptRoot "buffered_stream_capability_check.ps1") -StepArgs @(
        "-Port", $Port
    )
    Invoke-Step -Name "G-code Burst No Motion" -File (Join-Path $PSScriptRoot "gcode_burst_no_motion_check.ps1") -StepArgs @(
        "-Port", $Port,
        "-BurstLines", "6"
    )
    Invoke-Step -Name "G-code Stream Check" -File (Join-Path $PSScriptRoot "gcode_stream_check.ps1") -StepArgs @(
        "-Port", $Port
    )

    if (-not $SkipMotion) {
        Invoke-Step -Name "UI Control Matrix" -File (Join-Path $PSScriptRoot "ui_control_matrix_check.ps1") -StepArgs @(
            "-Port", $Port,
            "-FeedMmMin", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0}", $FeedMmMin))
        )
        Invoke-Step -Name "UI Trajectory Stress" -File (Join-Path $PSScriptRoot "ui_trajectory_stress.ps1") -StepArgs @(
            "-Port", $Port,
            "-Count", ([string]$Count),
            "-FeedMmMin", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0}", $FeedMmMin))
        )
        $csvPath = Join-Path $OutDir "gcode_feedback.csv"
        $summaryPath = Join-Path $OutDir "gcode_feedback_summary.csv"
        Invoke-Step -Name "G-code Feedback Error Stress" -File (Join-Path $PSScriptRoot "feedback_error_stress.ps1") -StepArgs @(
            "-Port", $Port,
            "-Count", ([string]$Count),
            "-FeedMmMin", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0}", $FeedMmMin)),
            "-MaxErrorMm", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0}", $MaxErrorMm)),
            "-CsvPath", $csvPath,
            "-EnableMotion"
        )
        Invoke-Step -Name "Feedback CSV Analysis" -File (Join-Path $PSScriptRoot "analyze_feedback_error_csv.ps1") -StepArgs @(
            "-CsvPath", $csvPath,
            "-MaxErrorMm", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0}", $MaxErrorMm)),
            "-MaxRmsMm", ([string]::Format([System.Globalization.CultureInfo]::InvariantCulture, "{0}", $MaxRmsMm)),
            "-WorstCount", "10",
            "-SummaryCsvPath", $summaryPath
        )
    }

    Write-Host ("FINAL_VALIDATION PASS out={0}" -f $OutDir)
    exit 0
} catch {
    Write-Host "FINAL_VALIDATION FAIL: $($_.Exception.Message)"
    exit 1
} finally {
    Pop-Location
}
