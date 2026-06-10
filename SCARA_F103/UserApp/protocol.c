#include "protocol.h"

/* 调试/安全文本协议：处理 VERSION、STATUS、HOSTCAP、ENABLE、STOP 等非 G-code 命令。 */

#include "app_config.h"
#include "app_params.h"
#include "binary_traj.h"
#include "gcode_stream.h"
#include "home_controller.h"
#include "home_sensor.h"
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
    Stepper_GetStateSnapshot(&s);
    HomeSensor_GetState(&home);

    SerialDma_SendFormat("STAT t=%lu m=%s e=%lu p=%ld,%ld "
                         "r=%u,%u en=%u,%u pps=%ld,%ld tgt=%ld,%ld wd=%u idle=%lu "
                         "rxov=%lu txd=%lu txq=%lu h=%u,%u hs=%s he=%u "
                         "bf=%u,%u q=%u sq=%u,%u jt=%s,%lu,%lu,%u,%u hz=%lu\r\n",
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
                         BinaryTraj_StateName(BinaryTraj_GetState()),
                         (unsigned long)BinaryTraj_AcceptedCount(),
                         (unsigned long)BinaryTraj_ExecutedCount(),
                         (unsigned int)BinaryTraj_BufferCount(),
                         (unsigned int)BinaryTraj_BufferFree(),
                         (unsigned long)APP_CONTROL_HZ);
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
        /* 告诉上位机：当前固件定位为“上位机规划、下位机执行”的脉冲控制器。 */
        const AppParams *p = AppParams_Get();
        SerialDma_SendFormat("OK HOSTCAP host_plan=1 gcode_abt=1 binary_traj=1 binary_timed=1 ptest=0 jogp=0 dda=1 hz=%lu ppr1=%ld ppr2=%ld\r\n",
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
            SerialDma_Send("ERR PPR_RANGE\r\n");
        } else if (Stepper_IsBusy() || GcodeStream_PlannerCount() > 0u) {
            SerialDma_Send("ERR PPR_BUSY\r\n");
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
    } else if (strcmp(cmd, "HOME_SENSOR") == 0) {
        HomeSensorState home;
        HomeSensor_GetState(&home);
        SerialDma_SendFormat("OK HOME_SENSOR h1=%u h2=%u mask=%u active_level=%u pins=PB0,PB1\r\n",
                             home.home1_active ? 1u : 0u,
                             home.home2_active ? 1u : 0u,
                             (unsigned int)home.active_mask,
                             (unsigned int)APP_HOME_SWITCH_ACTIVE_LEVEL);
    } else if (strcmp(cmd, "HOME") == 0 || strcmp(cmd, "HOME_REAL") == 0) {
        if (HomeController_Start(false)) {
            SerialDma_Send("OK HOME_REAL\r\n");
        } else {
            SerialDma_Send("ERR HOME_BUSY\r\n");
        }
    } else if (strcmp(cmd, "HOME_SIM") == 0) {
        if (HomeController_Start(true)) {
            SerialDma_Send("OK HOME_SIM\r\n");
        } else {
            SerialDma_Send("ERR HOME_BUSY\r\n");
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
            SerialDma_Send("ERR WATCHDOG\r\n");
        }
    } else if (strcmp(cmd, "WATCHDOG OFF") == 0) {
        s_watchdog_enabled = false;
        s_watchdog_tripped = false;
        SerialDma_Send("OK WATCHDOG OFF\r\n");
    } else if (strcmp(cmd, "STOP") == 0) {
        HomeController_Stop();
        BinaryTraj_Stop();
        GcodeStream_Clear();
        MotionPlanner_Stop();
        SerialDma_Send("OK STOP\r\n");
    } else if (strcmp(cmd, "ESTOP") == 0) {
        HomeController_Stop();
        BinaryTraj_Stop();
        GcodeStream_Clear();
        Stepper_EStopAll();
        SerialDma_Send("OK ESTOP\r\n");
    } else if (strcmp(cmd, "CLEAR_ERROR") == 0 || strcmp(cmd, "RESET") == 0) {
        Stepper_ClearError();
        HomeController_ClearError();
        s_watchdog_tripped = false;
        SerialDma_Send("OK CLEAR_ERROR\r\n");
    } else if (strcmp(cmd, "ZERO") == 0) {
        MotionPlanner_Stop();
        Stepper_Zero();
        SerialDma_Send("OK ZERO\r\n");
    } else if (strncmp(cmd, "JOGP", 4) == 0 || strncmp(cmd, "PTEST", 5) == 0) {
        SerialDma_Send("ERR USE_G1_ABT\r\n");
    } else if (sscanf(cmd, "ENABLE %ld", &a) == 1) {
        Stepper_EnableAll(a != 0);
        SerialDma_Send(a != 0 ? "OK ENABLE 1\r\n" : "OK ENABLE 0\r\n");
    } else {
        SerialDma_Send("ERR BAD_CMD\r\n");
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
