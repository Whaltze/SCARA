#include "app_main.h"

/* 用户主循环入口：CubeMX 初始化后进入这里，周期性处理串口、协议、运动和回零状态机。 */

#include "app_config.h"
#include "board_pins.h"
#include "app_params.h"
#include "gcode_stream.h"
#include "home_controller.h"
#include "laser_control.h"
#include "motion_planner.h"
#include "protocol.h"
#include "scara_kinematics.h"
#include "serial_dma.h"
#include "stepper_driver.h"

static volatile uint32_t s_max_tick_cycles;

void App_Init(void)
{
    LaserControl_Init();
    AppParams_Init();
    Stepper_Init();
    ScaraKinematics_Init();
    HomeController_Init();
    MotionPlanner_Init();
    GcodeStream_Init();
    Protocol_Init();
    SerialDma_Init();
    HAL_TIM_Base_Start_IT(BOARD_TICK_TIM);
    SerialDma_Send("BOOT " APP_FW_NAME " " APP_FW_VERSION " READY\r\n");
}

uint32_t App_MaxTickCycles(void)
{
    return s_max_tick_cycles;
}

void App_Loop(void)
{
    char line[APP_SERIAL_LINE_SIZE];
    char realtime;

    /* 主循环只做非中断重活：串口收包、协议解析、G-code 入队、回零流程推进。 */
    SerialDma_Poll();
    while (SerialDma_ReadRealtime(&realtime)) {
        line[0] = realtime;
        line[1] = '\0';
        Protocol_ProcessLine(line);
    }
    /*
     * Match Grbl's planner backpressure and fill behavior: leave complete
     * lines in the RX ring when planner/TX space is unavailable, otherwise
     * drain all acceptable lines before preparing segments. Parsing only one
     * line here lets a short block be consumed as an apparent program end
     * before its successor reaches the planner.
     */
    for (;;) {
        HomeControllerState home_state = HomeController_GetState();
        bool homing_active = home_state != HOME_CTRL_IDLE &&
                             home_state != HOME_CTRL_DONE &&
                             home_state != HOME_CTRL_ERROR;
        if (homing_active ||
            GcodeStream_HomeAckPending() ||
            GcodeStream_PlannerFree() == 0u ||
            SerialDma_TxFreeCount() == 0u ||
            !SerialDma_ReadLine(line, sizeof(line))) {
            break;
        }
        Protocol_ProcessLine(line);
    }
    HomeController_Loop();
    MotionPlanner_Loop();
    GcodeStream_Loop();
    Protocol_Loop();
}

void App_StepEventIrq(void)
{
    uint32_t tick_start = DWT->CYCCNT;

    Stepper_StepEventIrq();

    uint32_t tick_cycles = (uint32_t)(DWT->CYCCNT - tick_start);
    if (tick_cycles > s_max_tick_cycles) {
        s_max_tick_cycles = tick_cycles;
    }
}

void App_Tick1kHz(void)
{
    HomeController_Tick1kHz();
    MotionPlanner_Tick1kHz();
    GcodeStream_Tick1kHz();
    Protocol_Tick1kHz();
    StepperState stepper;
    Stepper_GetStateSnapshot(&stepper);
    uint32_t error = stepper.axis[0].error | stepper.axis[1].error;
    if (error != 0u || HomeController_GetState() == HOME_CTRL_ERROR) {
        LaserControl_Disarm();
    } else {
        bool controller_idle = !Stepper_IsBusy() &&
                               GcodeStream_PlannerCount() == 0u;
        LaserControl_Tick1kHz(controller_idle);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == BOARD_TICK_TIM) {
        App_StepEventIrq();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == BOARD_UART) {
        SerialDma_TxCpltCallback();
    }
}
