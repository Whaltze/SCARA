param([string]$Preset = "Debug")

$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$repoRoot = (Resolve-Path (Join-Path $root "..")).Path
$failures = 0

function Pass([string]$Message) { Write-Host "PASS $Message" }
function Fail([string]$Message) { Write-Host "FAIL $Message"; $script:failures++ }
function Require-Text([string]$Path, [string]$Pattern, [string]$Message) {
    $text = Get-Content -Raw $Path
    if ($text -match $Pattern) { Pass $Message } else { Fail $Message }
}
function Forbid-Text([string]$Path, [string]$Pattern, [string]$Message) {
    $text = Get-Content -Raw $Path
    if ($text -notmatch $Pattern) { Pass $Message } else { Fail $Message }
}

Push-Location $root
try {
    & cmake --build --preset $Preset
    if ($LASTEXITCODE -eq 0) { Pass "firmware builds" } else { Fail "firmware build failed" }
} finally {
    Pop-Location
}

$build = Join-Path $root "build\$Preset"
$bin = Join-Path $build "SCARA_F103.bin"
if (Test-Path $bin) {
    $size = (Get-Item $bin).Length
    if ($size -le (61 * 1024)) { Pass "Flash leaves at least 2KB: $size bytes" } else { Fail "Flash reserve below 2KB: $size bytes" }
} else {
    Fail "firmware binary missing"
}

$config = Join-Path $root "UserApp\app_config.h"
$planner = Join-Path $root "UserApp\motion_planner.c"
$gcode = Join-Path $root "UserApp\gcode_stream.c"
$homeController = Join-Path $root "UserApp\home_controller.c"
$protocol = Join-Path $root "UserApp\protocol.c"
$serial = Join-Path $root "UserApp\serial_dma.c"
$stepper = Join-Path $root "UserApp\stepper_driver.c"
$laser = Join-Path $root "UserApp\laser_control.c"
$timer = Join-Path $root "Core\Src\tim.c"
$irq = Join-Path $root "Core\Src\stm32f1xx_it.c"
$appMain = Join-Path $root "UserApp\app_main.c"
$cmake = Join-Path $root "CMakeLists.txt"
$uiMotion = Join-Path $repoRoot "SCARA_UI\motion\motion_mixin.py"
$uiSerial = Join-Path $repoRoot "SCARA_UI\communication\serial_mixin.py"
$uiSender = Join-Path $repoRoot "SCARA_UI\communication\motion_senders.py"
$uiPlotting = Join-Path $repoRoot "SCARA_UI\ui\plotting.py"
$uiMixin = Join-Path $repoRoot "SCARA_UI\ui\ui_mixin.py"
$uiProtocol = Join-Path $repoRoot "SCARA_UI\communication\serial_protocol.py"
$uiLookAhead = Join-Path $repoRoot "SCARA_UI\trajectory\look_ahead.py"
$uiVision = Join-Path $repoRoot "SCARA_UI\vision\vision_mixin.py"
$uiUtility = Join-Path $repoRoot "SCARA_UI\core\utility_mixin.py"

Require-Text $config "APP_GCODE_PLANNER_BLOCKS 48u" "planner has 48 short-block look-ahead blocks"
Require-Text $config "APP_GCODE_BLEND_MIN_BLOCKS 12u" "stream prefill collects useful look-ahead"
Require-Text $config "APP_GCODE_TAIL_HOLDBACK_SEGMENTS 8u" "short streams retain a tail look-ahead window"
Require-Text $config "APP_STEPPER_TIMED_SEGMENTS 16u" "step layer has 16 segments"
Require-Text $config "APP_SERIAL_RX_RING_SIZE 256u" "RX byte ring is 256 bytes"
Require-Text $config "APP_SERIAL_TX_SIZE 512u" "diagnostic status frames cannot truncate"
Require-Text $config "APP_GRBL_SEGMENT_MS 5u" "segments target 5ms"
Require-Text $planner "planner_recalculate" "planner has incremental look-ahead"
Require-Text $planner "junction_speed_sqr" "planner uses junction-deviation speed"
Require-Text $planner "joint_deviation_pulse" "SCARA junction speed is constrained in physical joint space"
Require-Text $planner "final_segment_ticks" "short block tails use variable segment duration"
Require-Text $planner "block->max_entry_speed_sqr = block->entry_speed_sqr" "replanning preserves prepared predecessor entry speed"
Require-Text $planner "ScaraKinematics_InverseUmToPulse" "segment preparation performs SCARA IK"
Require-Text $planner "validate_and_limit_path" "SCARA joint limits are applied before look-ahead"
Require-Text $planner "rate_limited_segment_count" "dense step segments are shortened and diagnosed"
Require-Text $planner "max_refill_gap_ms" "segment refill latency is diagnosed"
Require-Text $planner "final unknown-successor" "segment prep preserves streamed junction context"
Forbid-Text $planner "pps_quantum|dv1|dv2" "segment prep does not reject planned blocks by adjacent PPS delta"
Require-Text $planner "dwell_p1 = s_last_segment_valid" "dwell barriers hold the last prepared endpoint"
Require-Text $planner "MOTION_LASER_DYNAMIC" "planner supports dynamic laser mode"
Require-Text $gcode "line\[1\].*'J'" "G-code parser supports GRBL jog"
Require-Text $gcode "GcodeStream_PlannerFree" "parser exposes planner backpressure"
Require-Text $gcode "MPos:" "status reports MPos"
Require-Text $gcode "JPos:" "status reports JPos"
Require-Text $gcode "Q:" "status reports planner occupancy"
Require-Text $gcode "E:" "status reports stepper error bits"
Require-Text $gcode "Seg:" "status reports segment diagnostics"
Require-Text $gcode "Pf:" "status reports planner preparation faults"
Require-Text $gcode "Rl:" "status reports adaptive segment rate limiting"
Require-Text $gcode "Pg:" "status reports maximum segment refill gap"
Require-Text $gcode "HS:" "status reports homing state"
Require-Text $gcode "A1:" "status reports axis 1 diagnostics"
Require-Text $gcode "A2:" "status reports axis 2 diagnostics"
Require-Text $gcode "Lz:" "status reports laser diagnostics"
Require-Text $gcode "is_home_sim_command" "G-code `$H supports simulated homing"
Require-Text $gcode "s_home_pending_ack" "formal homing ACK waits for completion"
Require-Text $gcode "if \(MotionPlanner_IsBusy\(\)\)" "formal homing rejects pending planner motion"
Require-Text $appMain "homing_active" "normal G-code pauses while homing"
Require-Text $appMain "GcodeStream_HomeAckPending" "normal G-code waits for final homing ACK"
Require-Text $appMain "SerialDma_TxFreeCount\(\) == 0u" "normal G-code waits for an ACK TX slot"
Require-Text $appMain "for \(;;\)" "main loop fills the planner before segment preparation"
Require-Text $homeController "HOME_ERR_SWITCH_ACTIVE" "real homing rejects already-active HOME switches"
Require-Text $protocol "ERR HOME_SWITCH_ACTIVE" "text HOME reports active switch precheck"
Require-Text $serial "0x85u" "Jog Cancel bypasses normal parser"
Require-Text $stepper "TIM1->CR1 \|= TIM_CR1_CEN" "TIM1 emits non-blocking one-pulse STEP"
Require-Text $stepper "TIM4->CR1 \|= TIM_CR1_CEN" "TIM4 emits non-blocking one-pulse STEP"
Require-Text $stepper "__HAL_TIM_SET_AUTORELOAD\(BOARD_TICK_TIM" "TIM2 uses variable step-event periods"
Forbid-Text $stepper "delay_us\(APP_STEPPER_PULSE" "STEP pulse width does not block the ISR"
Require-Text $config "APP_BACKLASH_COMP_PPS" "backlash compensation pulse rate is configurable"
Require-Text $stepper "backlash_prepare_for_move" "stepper schedules backlash takeup on joint reversal"
Require-Text $stepper "s_engaged_dir" "stepper tracks per-motor engaged direction for backlash"
Require-Text $gcode '\$160=' "online per-motor backlash calibration command exists"
Require-Text $config "APP_LASER_RELAY_ACTIVE_LEVEL" "laser relay polarity is configurable for fail-safe-off"
Require-Text $laser "relay_write" "relay writes go through the polarity-aware helper"
Require-Text $gcode "MOTION_LASER_PREP" "M6 wires the relay pre-engage dwell"
Require-Text $uiSerial "M6 P" "UI pre-engages the relay before pen-down"
Require-Text $uiSerial "M3 S" "UI marks with constant-power M3 (no corner dropout)"
Require-Text $uiSerial "laser_trajectory_mode" "trajectory laser mode gates marking separately from ARM"
Require-Text $uiSender "laser_trajectory_mode" "job preamble marking is gated on trajectory laser mode"
Require-Text $uiMixin "laser_traj_toggle" "UI exposes a dedicated trajectory-laser-mode button"
Require-Text $timer "TIM_OCPOLARITY_LOW" "one-pulse polarity matches common-anode driver"
Require-Text $irq "App_Tick1kHz\(\)" "millisecond safety/status work runs from SysTick"
Require-Text $uiMotion 'load_gcode_job\(\["\$HS"\]\)' "UI simulated homing uses formal GRBL queue"
Require-Text $uiMotion 'load_gcode_job\(\["\$H"\]\)' "UI real homing uses formal GRBL queue"
Require-Text $uiSerial "_line_requires_homing" "UI distinguishes motion from setup and homing commands"
Require-Text $uiSerial "_preflight_motion_path" "UI preflights every formal motion path"
Require-Text $uiSerial "_line_is_long_running" "UI isolates long-running homing commands"
Require-Text $uiSerial "_abort_stalled_stream" "UI can recover a stuck ACK stream"
Require-Text $uiSerial "return 64, 224" "UI uses a fixed GRBL character-counting window"
Require-Text $uiSerial "previous_fault_count is not None" "UI establishes a planner-fault baseline"
Require-Text $uiUtility "Manual motion G-code is blocked" "manual text motion cannot bypass path preflight"
Require-Text $uiSerial "hs=.*he=.*en=.*pps=.*h=" "UI STATUS parser exposes homing diagnostics"
Require-Text $uiSerial '\$110=' "UI syncs selected speed cap"
Require-Text $uiSerial '\$120=' "UI syncs selected acceleration"
Require-Text $uiSerial '\$11=' "UI syncs junction deviation"
Require-Text $uiSerial '\$160=' "UI syncs per-motor backlash compensation"
Require-Text $uiMotion 'self\.ser\.write\(b"\\x18"\)' "UI stop uses GRBL Ctrl-X"
Require-Text $uiMotion "generate_stroke_motion" "writing uses compact geometry motion"
Require-Text $uiMotion 'yield "G4 P0\.001"' "writing stroke transitions use exact-stop barriers"
Require-Text $uiMotion "HANDWRITING_CORNER_RADIUS_MM = 0\.0" "handwriting preserves sharp G1 corners"
Require-Text $uiMotion "TEXT_CORNER_RADIUS_MM = 0\.0" "text outlines preserve sharp G1 corners"
Require-Text $uiMotion "safe non-zero transition speed" "writing uses GRBL junction speed at sharp corners"
Require-Text $uiMotion "Iterative RDP" "long writing simplification is non-recursive"
Require-Text $uiMotion "CountedCommandStream" "writing commands remain lazy with known progress"
Require-Text $uiMotion "_qt_path_contours" "text preserves semantic LineTo and CurveTo boundaries"
Forbid-Text $uiMotion "toSubpathPolygons" "text no longer loses corner semantics in polygon flattening"
Require-Text $uiSerial "mcu_planner_capacity" "UI uses the expanded controller planner capacity"
Require-Text $uiSerial 'mcu_planner_free \+ int\(q_match\.group\(1\)\)' "UI derives planner capacity from live status"
Forbid-Text $uiMotion "tuple\(self\._iter_stroke_geometry_gcode" "writing stream is not fully materialized"
Require-Text $uiMotion "CAR_CORNER_RADIUS_MM = 0\.0" "car outline preserves sharp control points"
Require-Text $uiMotion "PATH_SIMPLIFY_TOLERANCE_MM = 0\.0" "writing does not delete input control points"
Forbid-Text $uiMotion "_should_linearize_geometry_arc|_sample_linearized_arc_points" "true arcs are not rewritten as short G1 blocks"
Require-Text $uiMotion 'yield "G4 P0\.001"' "geometry connectors settle at their exact start point"
Require-Text $planner "distance_mm >= block->length_mm" "planner final segments use exact Cartesian endpoints"
Require-Text $uiPlotting "required\.update\(index for index, point in enumerate\(path\)" "preview decimation preserves geometry endpoints"
Require-Text $uiMixin "self\._append_event_point\(event\)" "handwriting records the release endpoint"
Require-Text $uiLookAhead '0\.5 \* \(1\.0 \+ dot\)' "UI junction half-angle matches GRBL"
Require-Text $uiSender "motion_profile_sync_requested" "motion profile precedes geometry stream"
Forbid-Text $uiProtocol "checksum|cs=" "UI ACK parser has no legacy checksum protocol"
Forbid-Text $uiSerial "process_host_segment|last_sent_cs|expected_checksum|rx_checksum" "UI sender has no old host-timed ACK state"
Forbid-Text $uiMotion '_segment_joint_feed_cap|_path_joint_accel_cap_mm_s2|yield "G61"|yield "G64"' "UI leaves writing transition speed to the MCU planner"
Forbid-Text $uiVision "generate_binary" "vision path uses the GRBL stream"
Forbid-Text $cmake "binary_traj\.c" "BinaryTraj is removed from firmware build"

if (Test-Path (Join-Path $root "UserApp\binary_traj.c")) { Fail "binary_traj.c still exists" } else { Pass "binary_traj.c deleted" }
if (Test-Path (Join-Path $root "UserApp\binary_traj.h")) { Fail "binary_traj.h still exists" } else { Pass "binary_traj.h deleted" }

if ($failures -gt 0) {
    throw "VERIFY_PROJECT failed: $failures checks"
}
Write-Host "VERIFY_PROJECT PASS"
