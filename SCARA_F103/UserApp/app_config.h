#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 固件集中调参文件。
 * 机械尺寸、10kHz 控制周期、速度/加速度、串口缓冲、回零和 Flash 参数都优先在这里调整。
 * 修改后必须重新编译并重新烧录；同时确认 UI 里的 PPR、零点和机构尺寸保持一致。
 */

#include <stdint.h>

/* Firmware identity shown by VERSION. */
#define APP_FW_NAME "SCARA_F103"
#define APP_FW_VERSION "0.26.3"

/* Current controller drives two stepper axes. */
#define APP_AXIS_COUNT 2u

/* Stepper driver ENA pins. Most optocoupled drivers enable when ENA is low. */
#define APP_STEPPER_ENA_ACTIVE_LEVEL 0u
#define APP_STEPPER_ENABLE_ON_BOOT 1u
/* Common-anode wiring: PUL+/DIR+/ENA+ to +5V, MCU drives PUL-/DIR-/ENA-.
 * The optocoupler is active when the MCU pin sinks low.
 */
#define APP_STEPPER_PUL_ACTIVE_LEVEL 0u
#define APP_STEPPER_M1_DIR_POSITIVE_LEVEL 1u
#define APP_STEPPER_M2_DIR_POSITIVE_LEVEL 0u
/* DM556 requires PUL high/low >= 2.5us and DIR setup >= 6us.
 * Delays use the Cortex-M3 DWT cycle counter, so these values are independent
 * of compiler optimization and remain bounded inside the 100us control tick.
 */
#define APP_STEPPER_PULSE_HIGH_US 3u
#define APP_STEPPER_PULSE_LOW_US 3u
#define APP_STEPPER_DIR_SETUP_US 6u

/* PWM 脉冲定时器基准频率，当前 1MHz 表示 1us 计数精度。 */
#define APP_STEPPER_TIMER_HZ 1000000u
/* 实时插补/速度更新频率。10000Hz 表示每 100us 执行一次运动内核。 */
#define APP_CONTROL_HZ 10000u
#define APP_CONTROL_PERIOD_US (1000000u / APP_CONTROL_HZ)
#define APP_STEPPER_ISR_DELAY_BUDGET_US (APP_CONTROL_PERIOD_US / 2u)
#define APP_STEPPER_WORST_BLOCKING_DELAY_US \
    ((APP_AXIS_COUNT * APP_STEPPER_DIR_SETUP_US) + APP_STEPPER_PULSE_HIGH_US + APP_STEPPER_PULSE_LOW_US)

#if APP_STEPPER_WORST_BLOCKING_DELAY_US > APP_STEPPER_ISR_DELAY_BUDGET_US
#error "Configured step pulse delays exceed half of the control ISR period"
#endif

/* 最低有效 PPS，低于该值按停止处理，防止很慢的抖动脉冲。 */
#define APP_MIN_EFFECTIVE_PPS 16
#define APP_INTERPOLATOR_MIN_PPS 1
/* 默认单轴速度上限。若 UI 默认 PPR 从 1600 提到 3200，同样 mm/s 会需要更高 PPS。 */
#define APP_MAX_PPS_DEFAULT 10000
/* 单轴 PPS 硬上限，只用于保护，实际运行应留足余量。 */
#define APP_MAX_PPS_HARD 50000

/* 默认加速度，单位 pulses/s^2；调大响应快但更容易振动/丢步。 */
#define APP_ACCEL_DEFAULT 200



/* 加速度上限，防止上位机或命令传入过激参数。 */
#define APP_ACCEL_MAX 5000

/* Software pulse position range. Only used when MCU-side limits are enabled. */
#define APP_MOTOR_MIN_POS (-2000000000L)
#define APP_MOTOR_MAX_POS (2000000000L)
/* 1: host PC owns workspace/path limit checks.
 * 0: MCU also enforces motor pulse range / joint range checks.
 */
#define APP_HOST_OWNS_LIMIT_CHECKS 1u

/* 电机和传动参数，用于脉冲与关节角换算；必须和上位机、驱动器拨码一致。 */
/* 驱动器细分后的每圈脉冲数。若 UI 使用 3200，这里或运行时 PPR 命令也应同步为 3200。 */
#define APP_PULSES_PER_REV_M1 3200L
#define APP_PULSES_PER_REV_M2 3200L
/* Gear reduction ratio from motor shaft to active arm. */
#define APP_REDUCER_RATIO_M1 1L
#define APP_REDUCER_RATIO_M2 1L
/* Positive/negative sign for each motor direction. Use 1 or -1. */
#define APP_MOTOR1_DIR_SIGN 1L
#define APP_MOTOR2_DIR_SIGN 1L
/* Joint zero offsets in mrad after homing / mechanical calibration. */
#define APP_MOTOR1_ZERO_MRAD 2251L
#define APP_MOTOR2_ZERO_MRAD 890L

/* SCARA geometry in micrometers.
 * Must match the upper computer kinematic model.
 */
/* Distance between the two base motor axes. */
#define APP_SCARA_BASE_UM 150000L
/* Left and right active arm lengths. */
#define APP_SCARA_ACTIVE1_UM 160000L
#define APP_SCARA_ACTIVE2_UM 160000L
/* Left and right passive arm lengths. */
#define APP_SCARA_PASSIVE1_UM 200000L
#define APP_SCARA_PASSIVE2_UM 200000L
/* Joint angle soft ranges in milliradians. */
#define APP_SCARA_THETA1_MIN_MRAD (-3142L)
#define APP_SCARA_THETA1_MAX_MRAD 3142L
#define APP_SCARA_THETA2_MIN_MRAD 0L
#define APP_SCARA_THETA2_MAX_MRAD 6283L
/* Elbow branch selection for inverse kinematics. Usually keep paired with UI. */
#define APP_SCARA_IK_LEFT_ELBOW_SIGN 1
#define APP_SCARA_IK_RIGHT_ELBOW_SIGN (-1)

/* Legacy stream/status pacing period. */
#define APP_STREAM_PERIOD_MS 100u
/* Communication watchdog default.
 * 0 disables watchdog on boot; can be enabled later by command.
 */
#define APP_COMM_WATCHDOG_DEFAULT_MS 0u

/* Serial buffers and protocol sizing. */
/* Raw UART RX DMA ring buffer size in bytes. */
#define APP_SERIAL_RX_DMA_SIZE 256u
/* Max accepted single ASCII command line length. */
#define APP_SERIAL_LINE_SIZE 96u
/* TX staging buffer size for one formatted response. */
#define APP_SERIAL_TX_SIZE 280u
/* Number of parsed RX lines that can wait in queue. */
#define APP_SERIAL_LINE_QUEUE_DEPTH 16u
/* Realtime characters bypass the normal line queue. */
#define APP_SERIAL_REALTIME_QUEUE_DEPTH 8u
/* Number of pending TX messages that can wait in queue. */
#define APP_SERIAL_TX_QUEUE_DEPTH 8u
/* UART baudrate. Must match the upper computer. */
#define APP_SERIAL_BAUDRATE 115200u

/* Homing switch and homing process parameters. */
/* 0 means switch active-low, 1 means active-high. */
#define APP_HOME_SWITCH_ACTIVE_LEVEL 0u
/* Search direction for each homing axis. Use 1 or -1. 
// 左侧红色主动臂 Axis1：从 90° 竖直回零时应向左找水平限位，即角度增大到 180°。
*/

// 建议增加一个回零专用加速度，例如：
#define APP_HOME_ACCEL_PPS_S 2000

#define APP_HOME_AXIS1_DIR (1)
// 右侧蓝色主动臂 Axis2：若 HOME2 安装在右水平位置，则从 90° 竖直向右找 0°，角度减小。
#define APP_HOME_AXIS2_DIR (-1)
/* Homing search speed in pulses per second. */
#define APP_HOME_SEARCH_PPS 100
/* Pull-off distance after switch trigger, in pulses. */
#define APP_HOME_BACKOFF_PULSE 200L
/* Debounce time that the switch must stay active. */
#define APP_HOME_DEBOUNCE_MS 20u
/* Maximum allowed homing duration before timeout. */
#define APP_HOME_TIMEOUT_MS 30000u
/* Joint angle references used by homing, in milliradians. 
两个臂的回零 轴1向着左边，轴2向着右边
左红臂水平回零点：向左水平 = 180° ≈ 3142 mrad。

mrad = deg × π / 180 × 1000
*/
// HORIZONTAL_MRAD：理论水平角度，仅用于仿真或参考
// SWITCH_MRAD：真实开关触发角度，用于实机回零标定
// VERTICAL_MRAD：回零后要回到的 90° 位置
#define APP_HOME_AXIS1_SWITCH_MRAD 3316L    /* 示例：左臂触发点约 190° */
#define APP_HOME_AXIS2_SWITCH_MRAD (-175L)   /* 示例：右臂触发点约 -5° */

#define APP_HOME_AXIS1_HORIZONTAL_MRAD 3142L
#define APP_HOME_AXIS2_HORIZONTAL_MRAD 0L
#define APP_HOME_AXIS1_VERTICAL_MRAD 1571L
#define APP_HOME_AXIS2_VERTICAL_MRAD 1571L

/* G-code stream planner parameters. */
/* Number of queued motion blocks kept by MCU. */
#define APP_GCODE_PLANNER_BLOCKS 32u
/* Timed DDA segments prefetched below the planner for queue-to-queue handoff. */
#define APP_STEPPER_TIMED_SEGMENTS 4u
/* Wait time for the first block so a second block can arrive for blending. */
#define APP_GCODE_BLEND_START_DELAY_MS 20u
/* Automatic status push period while streaming. */
#define APP_GCODE_STATUS_PERIOD_MS 200u
/* Default feed rate for G1 if host does not send F, in mm/min. */
#define APP_GCODE_DEFAULT_FEED_MM_MIN 300L
/* G0 rapid speed, expressed in pulses per second. */
#define APP_GCODE_RAPID_PPS APP_MAX_PPS_DEFAULT
/* Legacy linear segmentation length in micrometers. */
#define APP_GCODE_LINEAR_SEGMENT_UM 2000L

/* MOVL segmentation length in micrometers. */
#define APP_MOVL_SEGMENT_UM 2000L

/* Buffered binary trajectory protocol. */
#define APP_BINARY_TRAJ_VERSION 1u
#define APP_BINARY_TRAJ_MAX_PAYLOAD 244u
#define APP_BINARY_TRAJ_POINTS 128u
#define APP_BINARY_TRAJ_MIN_PREFILL 4u
#define APP_BINARY_TRAJ_START_DELAY_MS 20u

/* Flash parameter page definition. */
/* Last flash page reserved for persisted AppParams. */
#define APP_PARAM_FLASH_ADDR 0x0800F800u
/* Flash page size for STM32F103C8/CB family. */
#define APP_PARAM_FLASH_PAGE_SIZE 1024u
/* Magic tag used to validate saved parameter blocks. */
#define APP_PARAM_FLASH_MAGIC 0x53434152u
/* Increment when default parameter layout/meaning changes. */
#define APP_PARAM_FLASH_VERSION 5u

#endif
