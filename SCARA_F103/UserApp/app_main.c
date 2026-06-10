#include "app_main.h"

/* 用户主循环入口：CubeMX 初始化后进入这里，周期性处理串口、协议、运动和回零状态机。 */

#include "app_config.h"
#include "board_pins.h"
#include "app_params.h"
#include "binary_traj.h"
#include "gcode_stream.h"
#include "home_controller.h"
#include "protocol.h"
#include "scara_kinematics.h"
#include "serial_dma.h"
#include "stepper_driver.h"

static volatile uint32_t s_max_tick_cycles;

void App_Init(void)
{
    AppParams_Init();
    Stepper_Init();
    ScaraKinematics_Init();
    HomeController_Init();
    GcodeStream_Init();
    BinaryTraj_Init();
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
    if (SerialDma_ReadLine(line, sizeof(line))) {
        Protocol_ProcessLine(line);
    }
    HomeController_Loop();
    BinaryTraj_Loop();
    GcodeStream_Loop();
    Protocol_Loop();
}

void App_Tick10kHz(void)
{
    static uint8_t low_rate_divider;
    uint32_t tick_start = DWT->CYCCNT;

    Stepper_Tick10kHz();
    BinaryTraj_Tick10kHz();

    low_rate_divider++;
    if (low_rate_divider >= 10u) {
        low_rate_divider = 0;
        HomeController_Tick1kHz();
        GcodeStream_Tick1kHz();
        Protocol_Tick1kHz();
    }

    uint32_t tick_cycles = (uint32_t)(DWT->CYCCNT - tick_start);
    if (tick_cycles > s_max_tick_cycles) {
        s_max_tick_cycles = tick_cycles;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == BOARD_TICK_TIM) {
        App_Tick10kHz();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == BOARD_UART) {
        SerialDma_TxCpltCallback();
    }
}
