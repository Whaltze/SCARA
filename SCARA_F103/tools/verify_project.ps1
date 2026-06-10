param(
    [string]$Preset = "Debug"
)

# 项目自检脚本：构建固件、检查产物、确认课程设计版保留的脚本和文档存在。

$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$buildDir = Join-Path $projectRoot "build\$Preset"
$firmwareBase = Join-Path $buildDir "SCARA_F103"
$failures = 0

function Pass {
    param([string]$Message)
    Write-Host "PASS $Message"
}

function Fail {
    param([string]$Message)
    Write-Host "FAIL $Message"
    $script:failures++
}

function Require-File {
    param([string]$Path)
    if (Test-Path $Path -PathType Leaf) {
        Pass "file exists: $Path"
    } else {
        Fail "missing file: $Path"
    }
}

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Description
    )

    if (-not (Test-Path $Path -PathType Leaf)) {
        Fail "cannot check missing file: $Path"
        return
    }

    $text = Get-Content -Raw $Path
    if ($text -match $Pattern) {
        Pass $Description
    } else {
        Fail $Description
    }
}

function Require-DocFile {
    param(
        [string]$RelativePath,
        [string]$AbsoluteFallback
    )

    $relative = Join-Path $projectRoot $RelativePath
    if (Test-Path $relative -PathType Leaf) {
        Pass "file exists: $relative"
        return
    }

    if (Test-Path $AbsoluteFallback -PathType Leaf) {
        Pass "file exists: $AbsoluteFallback"
        return
    }

    Fail "missing file: $relative"
}

Write-Host "== Build =="
Push-Location $projectRoot
try {
    & cmake --build --preset $Preset
    if ($LASTEXITCODE -ne 0) {
        Fail "cmake build preset $Preset"
    } else {
        Pass "cmake build preset $Preset"
    }
} finally {
    Pop-Location
}

Write-Host "== Firmware Artifacts =="
Require-File "$firmwareBase.elf"
Require-File "$firmwareBase.hex"
Require-File "$firmwareBase.bin"
Require-File "$firmwareBase.map"

$binPath = "$firmwareBase.bin"
if (Test-Path $binPath -PathType Leaf) {
    $binSize = (Get-Item $binPath).Length
    $limit = 63 * 1024
    if ($binSize -le $limit) {
        Pass "binary size $binSize <= $limit bytes"
    } else {
        Fail "binary size $binSize > $limit bytes"
    }
}

Write-Host "== Flash Layout =="
$linker = Join-Path $projectRoot "STM32F103XX_FLASH.ld"
$config = Join-Path $projectRoot "UserApp\app_config.h"
Require-Text $linker "FLASH \(rx\)\s+:\s+ORIGIN = 0x8000000, LENGTH = 63K" "linker reserves final parameter page"
Require-Text $config "APP_PARAM_FLASH_ADDR 0x0800F800u" "parameter page address is 0x0800F800"
Require-Text $config "APP_FW_VERSION `"0\.26\.0`"" "firmware version is 0.26.0"
Require-Text $config "APP_CONTROL_HZ 10000u" "control loop is 10 kHz"
Require-Text $config "APP_MOTOR1_ZERO_MRAD 2251L" "motor 1 zero offset matches symmetric UI home"
Require-Text $config "APP_MOTOR2_ZERO_MRAD 890L" "motor 2 zero offset matches symmetric UI home"
Require-Text $config "APP_SERIAL_BAUDRATE 115200u" "serial baudrate is 115200"
Require-Text $config "APP_COMM_WATCHDOG_DEFAULT_MS 0u" "comm watchdog is disabled by default"
Require-Text $config "APP_HOST_OWNS_LIMIT_CHECKS 1u" "host owns trajectory limit checks"
Require-Text $config "APP_SCARA_IK_LEFT_ELBOW_SIGN 1" "left IK branch is non-crossed"
Require-Text $config "APP_SCARA_IK_RIGHT_ELBOW_SIGN \(-1\)" "right IK branch is non-crossed"
Require-Text $config "APP_PARAM_FLASH_VERSION 5u" "parameter flash version invalidates old zero offsets and PPR defaults"

$timer = Join-Path $projectRoot "Core\Src\tim.c"
Require-Text $timer "htim2\.Init\.Period = 99;" "TIM2 period is 99 for 10 kHz control tick"

$binaryTraj = Join-Path $projectRoot "UserApp\binary_traj.c"
Require-Text $binaryTraj "(?s)void BinaryTraj_Tick10kHz\(void\)\s*\{.*service_motion_10khz\(\);" "binary trajectory dispatch runs from 10 kHz tick"
Require-Text $binaryTraj "Stepper_IsBusy\(\) \|\| s_run_requested \|\| s_state == BINARY_TRAJ_STATE_RUNNING" "binary trajectory rejects BEGIN while running between segments"
Require-Text $binaryTraj "uint8_t payload\[32\]" "binary status includes interpolation diagnostics"
Require-Text $binaryTraj "exit1 = v1 < nv1 \? v1 : nv1" "binary trajectory blends exit speed against next segment"
Require-Text $binaryTraj "s_stream_underrun_count" "binary trajectory counts distinct stream underruns"
Require-Text $binaryTraj "BT_POINT_FLAG_HOST_TIMED" "binary trajectory supports buffered host-timed segments"
Require-Text $binaryTraj "Stepper_CanQueueTimedSegment\(\)" "binary host-timed segments prefill the step FIFO"
Require-Text $binaryTraj "Stepper_MoveAbsTicks\(point->p1_abs, point->p2_abs, point->v_dom_pps\)" "binary host-timed segments execute through timed DDA"
Require-Text $binaryTraj "s_max_dispatch_gap_ticks = 1u" "binary dispatch gap reports queue handoff ticks instead of segment duration"
Require-Text $binaryTraj "(?s)s_frame_type == BT_TYPE_ABORT.*BinaryTraj_Stop\(\).*MotionPlanner_Stop\(\)" "binary abort stops active and prefetched step segments"

$gcodeStream = Join-Path $projectRoot "UserApp\gcode_stream.c"
Require-Text $gcodeStream "JU:%lu,%lu,%u,%lu" "ASCII status reports binary trajectory underrun diagnostics"
Require-Text $gcodeStream "Sq:%u,%u" "ASCII status reports step segment queue occupancy"
Require-Text $gcodeStream "SerialDma_RxFreeBytes\(\)" "ASCII status reports RX byte capacity"

$stepper = Join-Path $projectRoot "UserApp\stepper_driver.c"
Require-Text $stepper "s_timed_segments\[APP_STEPPER_TIMED_SEGMENTS\]" "stepper has a timed segment FIFO"
Require-Text $stepper "timed_queue_pop\(&next\)" "stepper performs ISR-side queue-to-queue handoff"
Require-Text $stepper "timed_move_start_locked\(next\.pos1, next\.pos2, next\.duration_ticks\)" "ISR handoff starts timed segment without public API interrupt restore"
Require-Text $stepper "s_move\.host_timed && s_timed_count < APP_STEPPER_TIMED_SEGMENTS" "stepper accepts timed prefetch while running"
Require-Text $gcodeStream "block->timed \? Stepper_CanQueueTimedSegment\(\) : Stepper_CanAcceptMove\(\)" "G-code dispatch only prefetches timed segments"
Require-Text $gcodeStream "(?s)if \(line\[0\] == '!'\).*BinaryTraj_Stop\(\).*MotionPlanner_Stop\(\)" "real-time hold stops buffered binary motion"
Require-Text $gcodeStream "(?s)line\[0\] == 0x18u\).*BinaryTraj_Stop\(\).*MotionPlanner_Stop\(\)" "soft reset stops buffered binary motion"

Write-Host "== Build Rules and VS Code =="
$cmake = Join-Path $projectRoot "CMakeLists.txt"
Require-Text $cmake "-O ihex" "CMake generates hex"
Require-Text $cmake "-O binary" "CMake generates bin"
Require-Text $cmake "UserApp/gcode_stream\.c" "CMake builds gcode stream"
Require-Text $cmake "UserApp/binary_traj\.c" "CMake builds binary joint trajectory"
Require-Text $cmake "UserApp/home_controller\.c" "CMake builds home controller"
Require-Text $cmake "UserApp/home_sensor\.c" "CMake builds home sensor"
Require-Text $cmake "(?s)^(?!.*UserApp/pulse_protocol\.c)" "CMake does not build old pulse protocol"
Require-Text $cmake "(?s)^(?!.*UserApp/trajectory\.c)" "CMake does not build old trajectory queue"
Require-Text $cmake "(?s)^(?!.*UserApp/teach\.c)" "CMake does not build old teach module"
Require-File (Join-Path $projectRoot ".vscode\tasks.json")
Require-File (Join-Path $projectRoot ".vscode\launch.json")
Require-File (Join-Path $projectRoot ".vscode\settings.json")
Require-File (Join-Path $projectRoot "Run_Serial_Test.bat")
Require-File (Join-Path $projectRoot "Run_COM13_HostPlanned_3000.bat")
Require-File (Join-Path $projectRoot "tools\serial_link_check.ps1")
Require-File (Join-Path $projectRoot "tools\home_sensor_check.ps1")
Require-File (Join-Path $projectRoot "tools\gcode_stream_check.ps1")
Require-File (Join-Path $projectRoot "tools\buffered_stream_capability_check.ps1")
Require-File (Join-Path $projectRoot "tools\gcode_burst_no_motion_check.ps1")
Require-File (Join-Path $projectRoot "tools\flash_and_verify_buffered.ps1")
Require-File (Join-Path $projectRoot "tools\binary_joint_traj_stress.ps1")
Require-File (Join-Path $projectRoot "tools\ui_binary_line_stress.ps1")
Require-File (Join-Path $projectRoot "tools\ui_binary_car_stress.ps1")
Require-File (Join-Path $projectRoot "tools\feedback_error_stress.ps1")
Require-File (Join-Path $projectRoot "tools\analyze_feedback_error_csv.ps1")
Require-File (Join-Path $projectRoot "tools\sweep_binary_feedback_error.ps1")
Require-File (Join-Path $projectRoot "tools\simulate_binary_interpolator.ps1")
Require-File (Join-Path $projectRoot "tools\final_validation.ps1")
Require-File (Join-Path $projectRoot "tools\host_planned_stream_stress.ps1")
Require-File (Join-Path $projectRoot "tools\ui_control_matrix_check.ps1")
Require-File (Join-Path $projectRoot "tools\ui_trajectory_stress.ps1")
Require-DocFile "..\SCARA_UI\V_monitor.py" "C:\Users\22602\Desktop\SCARA\SCARA_UI\V_monitor.py"
Require-DocFile "..\SCARA_UI\tests\trajectory_planner_check.py" "C:\Users\22602\Desktop\SCARA\SCARA_UI\tests\trajectory_planner_check.py"
Require-DocFile "..\SCARA_UI\tests\sender_strategy_check.py" "C:\Users\22602\Desktop\SCARA\SCARA_UI\tests\sender_strategy_check.py"
Require-DocFile "..\SCARA_UI\tests\sender_benchmark_check.py" "C:\Users\22602\Desktop\SCARA\SCARA_UI\tests\sender_benchmark_check.py"
Require-DocFile "..\SCARA_UI\tests\feedback_error_check.py" "C:\Users\22602\Desktop\SCARA\SCARA_UI\tests\feedback_error_check.py"

$senders = Join-Path $projectRoot "..\SCARA_UI\communication\motion_senders.py"
Require-Text $senders "class AsciiLegacyG1Sender" "UI defines legacy G-code sender strategy"
Require-Text $senders "class BufferedBinarySender" "UI defines buffered binary sender strategy"
Require-Text $senders "class HostTimedSegmentSender" "UI defines host timed sender strategy"
$uiMixin = Join-Path $projectRoot "..\SCARA_UI\ui\ui_mixin.py"
Require-Text $uiMixin "lbl_sender_mode" "UI displays active sender mode and statistics"

$binaryStress = Join-Path $projectRoot "tools\binary_joint_traj_stress.ps1"
Require-Text $binaryStress "BINARY_JOINT_DIAG" "binary trajectory stress reports underrun diagnostics"

Write-Host "== Documentation =="
Require-DocFile "..\Version.md" "C:\Users\22602\Desktop\SCARA\Version.md"
Require-DocFile "..\Control.md" "C:\Users\22602\Desktop\SCARA\Control.md"
Require-DocFile "..\Work.md" "C:\Users\22602\Desktop\SCARA\Work.md"

if ($failures -eq 0) {
    Write-Host "VERIFY PASS"
    exit 0
}

Write-Host "VERIFY FAIL failures=$failures"
exit 1
