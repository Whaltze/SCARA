#include "protocol.h"

/* 调试/安全文本协议：处理 VERSION、STATUS、HOSTCAP、ENABLE、STOP 等非 G-code 命令。 */

#include "app_config.h"
#include "app_main.h"
#include "app_params.h"
#include "gcode_stream.h"
#include "home_controller.h"
#include "home_sensor.h"
#include "laser_control.h"
#include "motion_planner.h"
#include "serial_dma.h"
#include "stepper_driver.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static bool s_watchdog_enabled;
static uint32_t s_watchdog_timeout_ms;
static uint32_t s_watchdog_last_rx_ms;
static uint32_t s_protocol_tick_ms;
static bool s_watchdog_tripped;

#define HOME_ERR_SWITCH_ACTIVE_CODE 4u

static void send_protocol_error(const char *message)
{
    LaserControl_Disarm();
    SerialDma_Send(message);
}

static void upper_token(char *text)
{
    while (*text != '\0') {
        *text = (char)toupper((unsigned char)*text);
        text++;
    }
}

void Protocol_Init(void)
{
    s_watchdog_enabled = APP_COMM_WATCHDOG_DEFAULT_MS > 0u;
    s_watchdog_timeout_ms = APP_COMM_WATCHDOG_DEFAULT_MS;
    s_watchdog_last_rx_ms = 0;
    s_protocol_tick_ms = 0;
    s_watchdog_tripped = false;
}

void Protocol_Loop(void)
{
}

void Protocol_SendStatus(void)
{
    /* STATUS 是人工调试用长状态；自动 UI 状态主要由 gcode_stream.c 的 <...> 推送提供。 */
    StepperState s;
    HomeSensorState home;
    MotionPlannerSnapshot planner;
    LaserControlState laser;
    Stepper_GetStateSnapshot(&s);
    HomeSensor_GetState(&home);
    MotionPlanner_GetSnapshot(&planner);
    LaserControl_GetState(&laser);

    SerialDma_SendFormat("STAT t=%lu m=%s e=%lu p=%ld,%ld "
                         "r=%u,%u en=%u,%u pps=%ld,%ld tgt=%ld,%ld wd=%u idle=%lu "
                         "rxov=%lu txd=%lu txq=%lu h=%u,%u hs=%s he=%u "
                         "bf=%u,%u q=%u sq=%u,%u low=%u und=%lu pf=%lu rl=%lu gap=%lu prep=%lu done=%lu hz=%lu ic=%lu "
                         "laser=%u,%u,%u,%u\r\n",
                         (unsigned long)s.tick_ms,
                         Stepper_ModeName(s.axis[0].mode != STEPPER_MODE_IDLE ? s.axis[0].mode : s.axis[1].mode),
                         (unsigned long)(s.axis[0].error | s.axis[1].error),
                         (long)s.axis[0].position_pulse,
                         (long)s.axis[1].position_pulse,
                         s.axis[0].running ? 1u : 0u,
                         s.axis[1].running ? 1u : 0u,
                         s.axis[0].enabled ? 1u : 0u,
                         s.axis[1].enabled ? 1u : 0u,
                         (long)s.axis[0].current_pps,
                         (long)s.axis[1].current_pps,
                         (long)s.axis[0].target_pps,
                         (long)s.axis[1].target_pps,
                         s_watchdog_enabled ? 1u : 0u,
                         (unsigned long)(s_protocol_tick_ms - s_watchdog_last_rx_ms),
                         (unsigned long)SerialDma_RxOverflowCount(),
                         (unsigned long)SerialDma_TxDropCount(),
                         (unsigned long)SerialDma_TxQueuedCount(),
                         home.home1_active ? 1u : 0u,
                         home.home2_active ? 1u : 0u,
                         HomeController_StateName(HomeController_GetState()),
                         (unsigned int)HomeController_Error(),
                         (unsigned int)GcodeStream_PlannerFree(),
                         (unsigned int)SerialDma_RxFreeBytes(),
                         (unsigned int)GcodeStream_PlannerCount(),
                         (unsigned int)Stepper_TimedSegmentCount(),
                         (unsigned int)Stepper_TimedSegmentFree(),
                         (unsigned int)planner.segment_low_water,
                         (unsigned long)planner.segment_underrun_count,
                         (unsigned long)planner.preparation_fault_count,
                         (unsigned long)planner.rate_limited_segment_count,
                         (unsigned long)planner.max_refill_gap_ms,
                         (unsigned long)planner.prepared_segments,
                         (unsigned long)planner.completed_blocks,
                         (unsigned long)APP_CONTROL_HZ,
                         (unsigned long)App_MaxTickCycles(),
                         laser.armed ? 1u : 0u,
                         laser.relay_ready ? 1u : 0u,
                         laser.marking ? 1u : 0u,
                         (unsigned int)laser.power_permille);
}

static void send_errors(void)
{
    StepperState s;
    Stepper_GetStateSnapshot(&s);
    uint32_t err = s.axis[0].error | s.axis[1].error;

    SerialDma_SendFormat("ERRORS err=%lu soft_limit=%u estop=%u comm_timeout=%u disabled=%u\r\n",
                         (unsigned long)err,
                         (err & STEPPER_ERR_SOFT_LIMIT) ? 1u : 0u,
                         (err & STEPPER_ERR_ESTOP) ? 1u : 0u,
                         (err & STEPPER_ERR_COMM_TIMEOUT) ? 1u : 0u,
                         (err & STEPPER_ERR_DISABLED) ? 1u : 0u);
}

void Protocol_ProcessLine(const char *line)
{
    char cmd[APP_SERIAL_LINE_SIZE];
    long a = 0;
    long b = 0;

    if (line == NULL || line[0] == '\0') {
        return;
    }

    /* 任意有效串口输入都会刷新看门狗时间；默认看门狗已关闭，课程调试更清爽。 */
    s_watchdog_last_rx_ms = s_protocol_tick_ms;

    if (GcodeStream_TryProcessLine(line)) {
        return;
    }

    strncpy(cmd, line, sizeof(cmd) - 1u);
    cmd[sizeof(cmd) - 1u] = '\0';
    upper_token(cmd);

    if (strcmp(cmd, "PING") == 0) {
        SerialDma_Send("OK PONG\r\n");
    } else if (strcmp(cmd, "VERSION") == 0) {
        SerialDma_SendFormat("OK VERSION name=%s version=%s app_flash=63K baud=%lu\r\n",
                             APP_FW_NAME,
                             APP_FW_VERSION,
                             (unsigned long)APP_SERIAL_BAUDRATE);
    } else if (strcmp(cmd, "HOSTCAP") == 0) {
        /* 告诉上位机：UI 发送 G-code，MCU 负责 planner、segment preparation 和 STEP 输出。 */
        const AppParams *p = AppParams_Get();
        SerialDma_SendFormat("OK HOSTCAP grbl_stream=1 char_count=1 scara_plan=1 planner=%u segments=%u gcode_arc=1 jog=1 laser=%u laser_m3m4m5=1 dda=1 hz=%lu ppr1=%ld ppr2=%ld\r\n",
                             (unsigned int)APP_GCODE_PLANNER_BLOCKS,
                             (unsigned int)APP_STEPPER_TIMED_SEGMENTS,
                             (unsigned int)APP_LASER_COMMISSIONED,
                             (unsigned long)APP_CONTROL_HZ,
                             (long)p->pulses_per_rev[0],
                             (long)p->pulses_per_rev[1]);
    } else if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "QSTAT") == 0) {
        Protocol_SendStatus();
    } else if (strcmp(cmd, "PARAMS") == 0) {
        const AppParams *p = AppParams_Get();
        SerialDma_SendFormat("OK PARAMS ppr1=%ld ppr2=%ld base_um=%ld active_um=%ld,%ld passive_um=%ld,%ld\r\n",
                             (long)p->pulses_per_rev[0],
                             (long)p->pulses_per_rev[1],
                             (long)p->scara_base_um,
                             (long)p->active_arm_um[0],
                             (long)p->active_arm_um[1],
                             (long)p->passive_arm_um[0],
                             (long)p->passive_arm_um[1]);
    } else if (sscanf(cmd, "PPR %ld %ld", &a, &b) == 2 || sscanf(cmd, "PPR %ld", &a) == 1) {
        if (b <= 0) {
            b = a;
        }
        if (a <= 0 || b <= 0) {
            send_protocol_error("ERR PPR_RANGE\r\n");
        } else if (Stepper_IsBusy() || GcodeStream_PlannerCount() > 0u) {
            send_protocol_error("ERR PPR_BUSY\r\n");
        } else {
            AppParams *p = AppParams_Mutable();
            p->pulses_per_rev[0] = (int32_t)a;
            p->pulses_per_rev[1] = (int32_t)b;
            SerialDma_SendFormat("OK PPR ppr1=%ld ppr2=%ld\r\n", a, b);
        }
    } else if (sscanf(cmd, "HEARTBEAT %ld", &a) == 1) {
        StepperState s;
        Stepper_GetStateSnapshot(&s);
        SerialDma_SendFormat("OK HEARTBEAT seq=%ld tick=%lu err=%lu motion=%s home=%s gbuf=%u,%u\r\n",
                             a,
                             (unsigned long)s.tick_ms,
                             (unsigned long)(s.axis[0].error | s.axis[1].error),
                             Stepper_IsBusy() ? "Run" : "Idle",
                             HomeController_StateName(HomeController_GetState()),
                             (unsigned int)GcodeStream_PlannerFree(),
                             (unsigned int)GcodeStream_PlannerCount());
    } else if (strcmp(cmd, "ERRORS") == 0) {
        send_errors();
    } else if (strcmp(cmd, "LASER STATUS") == 0) {
        LaserControlState laser;
        LaserControl_GetState(&laser);
        uint8_t safety = (laser.pwm_ready ? 1u : 0u) |
                         (laser.boot_ready ? 2u : 0u) |
                         (APP_LASER_COMMISSIONED ? 4u : 0u) |
                         (APP_LASER_PWM_ACTIVE_HIGH ? 8u : 0u);
        SerialDma_SendFormat("OK LASER armed=%u ready=%u marking=%u power_permille=%u safety=%u\r\n",
                             laser.armed ? 1u : 0u,
                             laser.relay_ready ? 1u : 0u,
                             laser.marking ? 1u : 0u,
                             (unsigned int)laser.power_permille,
                             (unsigned int)safety);
    } else if (sscanf(cmd, "LASER POWER %ld", &a) == 1) {
        if (a < 0 || a > 65535 || !LaserControl_SetPowerPermille((uint16_t)a)) {
            send_protocol_error("ERR LASER_POWER_RANGE\r\n");
        } else {
            SerialDma_SendFormat("OK LASER POWER %ld\r\n", a);
        }
    } else if (strcmp(cmd, "LASER ARM") == 0) {
        HomeControllerState home_state = HomeController_GetState();
        if (Stepper_IsBusy() || GcodeStream_PlannerCount() > 0u ||
            (home_state != HOME_CTRL_IDLE && home_state != HOME_CTRL_DONE && home_state != HOME_CTRL_ERROR)) {
            send_protocol_error("ERR LASER_BUSY\r\n");
        } else if (!LaserControl_Arm()) {
            send_protocol_error("ERR LASER_NOT_READY\r\n");
        } else {
            SerialDma_Send("OK LASER ARM\r\n");
        }
    } else if (strcmp(cmd, "LASER DISARM") == 0) {
        LaserControl_Disarm();
        SerialDma_Send("OK LASER DISARM\r\n");
    } else if (strcmp(cmd, "HOME_SENSOR") == 0) {
        HomeSensorState home;
        HomeSensor_GetState(&home);
        SerialDma_SendFormat("OK HOME_SENSOR h1=%u h2=%u mask=%u active_level=%u pins=PB0,PB1\r\n",
                             home.home1_active ? 1u : 0u,
                             home.home2_active ? 1u : 0u,
                             (unsigned int)home.active_mask,
                             (unsigned int)APP_HOME_SWITCH_ACTIVE_LEVEL);
    } else if (strcmp(cmd, "HOME") == 0 || strcmp(cmd, "HOME_REAL") == 0) {
        if (MotionPlanner_IsBusy()) {
            send_protocol_error("ERR HOME_BUSY\r\n");
        } else if (HomeController_Start(false)) {
            MotionPlanner_Clear();
            SerialDma_Send("OK HOME_REAL\r\n");
        } else if (HomeController_Error() == HOME_ERR_SWITCH_ACTIVE_CODE) {
            send_protocol_error("ERR HOME_SWITCH_ACTIVE\r\n");
        } else {
            send_protocol_error("ERR HOME_BUSY\r\n");
        }
    } else if (strcmp(cmd, "HOME_SIM") == 0) {
        if (MotionPlanner_IsBusy()) {
            send_protocol_error("ERR HOME_BUSY\r\n");
        } else if (HomeController_Start(true)) {
            MotionPlanner_Clear();
            SerialDma_Send("OK HOME_SIM\r\n");
        } else if (HomeController_Error() == HOME_ERR_SWITCH_ACTIVE_CODE) {
            send_protocol_error("ERR HOME_SWITCH_ACTIVE\r\n");
        } else {
            send_protocol_error("ERR HOME_BUSY\r\n");
        }
    } else if (strcmp(cmd, "WATCHDOG") == 0) {
        SerialDma_SendFormat("OK WATCHDOG enabled=%u timeout_ms=%lu idle_ms=%lu tripped=%u\r\n",
                             s_watchdog_enabled ? 1u : 0u,
                             (unsigned long)s_watchdog_timeout_ms,
                             (unsigned long)(s_protocol_tick_ms - s_watchdog_last_rx_ms),
                             s_watchdog_tripped ? 1u : 0u);
    } else if (sscanf(cmd, "WATCHDOG ON %ld", &a) == 1) {
        if (a > 0) {
            s_watchdog_enabled = true;
            s_watchdog_timeout_ms = (uint32_t)a;
            s_watchdog_last_rx_ms = s_protocol_tick_ms;
            s_watchdog_tripped = false;
            SerialDma_Send("OK WATCHDOG ON\r\n");
        } else {
            send_protocol_error("ERR WATCHDOG\r\n");
        }
    } else if (strcmp(cmd, "WATCHDOG OFF") == 0) {
        s_watchdog_enabled = false;
        s_watchdog_tripped = false;
        SerialDma_Send("OK WATCHDOG OFF\r\n");
    } else if (strcmp(cmd, "STOP") == 0) {
        HomeController_Stop();
        GcodeStream_Clear();
        MotionPlanner_Stop();
        SerialDma_Send("OK STOP\r\n");
    } else if (strcmp(cmd, "ESTOP") == 0) {
        HomeController_Stop();
        GcodeStream_Clear();
        Stepper_EStopAll();
        SerialDma_Send("OK ESTOP\r\n");
    } else if (strcmp(cmd, "CLEAR_ERROR") == 0 || strcmp(cmd, "RESET") == 0) {
        if (strcmp(cmd, "RESET") == 0) {
            LaserControl_Disarm();
        }
        Stepper_ClearError();
        HomeController_ClearError();
        s_watchdog_tripped = false;
        SerialDma_Send("OK CLEAR_ERROR\r\n");
    } else if (strcmp(cmd, "ZERO") == 0) {
        MotionPlanner_Stop();
        Stepper_Zero();
        SerialDma_Send("OK ZERO\r\n");
    } else if (sscanf(cmd, "ENABLE %ld", &a) == 1) {
        if (a == 0) {
            LaserControl_Disarm();
        }
        Stepper_EnableAll(a != 0);
        SerialDma_Send(a != 0 ? "OK ENABLE 1\r\n" : "OK ENABLE 0\r\n");
    } else {
        send_protocol_error("ERR BAD_CMD\r\n");
    }
}

void Protocol_Tick1kHz(void)
{
    s_protocol_tick_ms++;

    if (s_watchdog_enabled &&
        s_watchdog_timeout_ms > 0u &&
        !s_watchdog_tripped &&
        (s_protocol_tick_ms - s_watchdog_last_rx_ms) > s_watchdog_timeout_ms &&
        Stepper_IsBusy()) {
        MotionPlanner_Stop();
        Stepper_SetErrorAll(STEPPER_ERR_COMM_TIMEOUT);
        s_watchdog_tripped = true;
    }
}
