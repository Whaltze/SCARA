#include "laser_control.h"

#include "app_config.h"
#include "board_pins.h"

static LaserControlState s_state;
static uint32_t s_idle_elapsed_ms;
static uint32_t s_boot_elapsed_ms;

static uint32_t pwm_compare(uint16_t power_permille)
{
    return APP_LASER_PWM_ACTIVE_HIGH
               ? (uint32_t)power_permille
               : APP_LASER_PWM_PERIOD_COUNTS - (uint32_t)power_permille;
}

static void force_safe_gpio_output(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
#if APP_LASER_PWM_ACTIVE_HIGH
    LASER_PWM_GPIO_Port->BRR = LASER_PWM_Pin;
#else
    LASER_PWM_GPIO_Port->BSRR = LASER_PWM_Pin;
#endif
    /* STM32F103 PA7 CRL nibble: 2 MHz general-purpose push-pull output. */
    LASER_PWM_GPIO_Port->CRL =
        (LASER_PWM_GPIO_Port->CRL & ~(0xFu << 28u)) | (0x2u << 28u);
}

static void select_pwm_output(void)
{
    /* STM32F103 PA7 CRL nibble: 50 MHz alternate-function push-pull. */
    LASER_PWM_GPIO_Port->CRL =
        (LASER_PWM_GPIO_Port->CRL & ~(0xFu << 28u)) | (0xBu << 28u);
}

static void pwm_off(void)
{
    /* Disconnect the timer from the pin before stopping it. This keeps the
     * physical PWM input at a defined off level even during timer transients.
     */
    force_safe_gpio_output();
    (void)HAL_TIM_PWM_Stop(BOARD_LASER_TIM, BOARD_LASER_TIM_CHANNEL);
    __HAL_TIM_SET_COMPARE(BOARD_LASER_TIM, BOARD_LASER_TIM_CHANNEL, APP_LASER_PWM_OFF_COMPARE);
    s_state.marking = false;
}

static bool pwm_start_marking(uint16_t power_permille)
{
    force_safe_gpio_output();
    __HAL_TIM_SET_COUNTER(BOARD_LASER_TIM, 0u);
    __HAL_TIM_SET_COMPARE(BOARD_LASER_TIM, BOARD_LASER_TIM_CHANNEL, pwm_compare(power_permille));
    if (HAL_TIM_PWM_Start(BOARD_LASER_TIM, BOARD_LASER_TIM_CHANNEL) != HAL_OK) {
        pwm_off();
        return false;
    }
    select_pwm_output();
    return true;
}

void LaserControl_EarlySafeOutput(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    BOARD_LASER_RELAY_PORT->BRR = BOARD_LASER_RELAY_PIN;
    force_safe_gpio_output();
    /* STM32F103 PA2 CRL nibble: 2 MHz general-purpose push-pull output. */
    BOARD_LASER_RELAY_PORT->CRL =
        (BOARD_LASER_RELAY_PORT->CRL & ~(0xFu << 8u)) | (0x2u << 8u);
}

void LaserControl_Init(void)
{
    s_state.armed = false;
    s_state.relay_ready = false;
    s_state.marking = false;
    s_state.task_started = false;
    s_state.pwm_ready = false;
    s_state.boot_ready = false;
    s_state.power_permille = APP_LASER_POWER_DEFAULT_PERMILLE;
    s_idle_elapsed_ms = 0u;
    s_boot_elapsed_ms = 0u;
    HAL_GPIO_WritePin(BOARD_LASER_RELAY_PORT, BOARD_LASER_RELAY_PIN, GPIO_PIN_RESET);
    pwm_off();
    if (APP_LASER_COMMISSIONED != 0u) {
        s_state.pwm_ready = true;
    }
}

bool LaserControl_SetPowerPermille(uint16_t power_permille)
{
    if (power_permille < APP_LASER_POWER_MIN_PERMILLE ||
        power_permille > APP_LASER_POWER_MAX_PERMILLE) {
        return false;
    }
    s_state.power_permille = power_permille;
    if (s_state.marking) {
        __HAL_TIM_SET_COMPARE(BOARD_LASER_TIM, BOARD_LASER_TIM_CHANNEL, pwm_compare(power_permille));
    }
    return true;
}

bool LaserControl_Arm(void)
{
    pwm_off();
    if (!s_state.pwm_ready || !s_state.boot_ready) {
        HAL_GPIO_WritePin(BOARD_LASER_RELAY_PORT, BOARD_LASER_RELAY_PIN, GPIO_PIN_RESET);
        s_state.armed = false;
        s_state.relay_ready = false;
        s_state.task_started = false;
        return false;
    }
    s_state.armed = true;
    s_state.relay_ready = false;
    s_state.task_started = false;
    s_idle_elapsed_ms = 0u;
    HAL_GPIO_WritePin(BOARD_LASER_RELAY_PORT, BOARD_LASER_RELAY_PIN, GPIO_PIN_RESET);
    return true;
}

void LaserControl_Disarm(void)
{
    pwm_off();
    HAL_GPIO_WritePin(BOARD_LASER_RELAY_PORT, BOARD_LASER_RELAY_PIN, GPIO_PIN_RESET);
    s_state.armed = false;
    s_state.relay_ready = false;
    s_state.task_started = false;
    s_idle_elapsed_ms = 0u;
}

bool LaserControl_CanMove(void)
{
    return true;
}

void LaserControl_BeginSegment(bool marking, bool prep)
{
    if (!s_state.armed) {
        pwm_off();
        HAL_GPIO_WritePin(BOARD_LASER_RELAY_PORT, BOARD_LASER_RELAY_PIN, GPIO_PIN_RESET);
        s_state.relay_ready = false;
        return;
    }
    s_state.task_started = true;
    s_idle_elapsed_ms = 0u;
    if (marking) {
        HAL_GPIO_WritePin(BOARD_LASER_RELAY_PORT, BOARD_LASER_RELAY_PIN, GPIO_PIN_SET);
        s_state.relay_ready = true;
        if (pwm_start_marking(s_state.power_permille)) {
            s_state.marking = true;
        } else {
            LaserControl_Disarm();
            s_state.pwm_ready = false;
        }
    } else if (prep) {
        pwm_off();
        HAL_GPIO_WritePin(BOARD_LASER_RELAY_PORT, BOARD_LASER_RELAY_PIN, GPIO_PIN_SET);
        s_state.relay_ready = true;
    } else {
        pwm_off();
        HAL_GPIO_WritePin(BOARD_LASER_RELAY_PORT, BOARD_LASER_RELAY_PIN, GPIO_PIN_RESET);
        s_state.relay_ready = false;
    }
}

void LaserControl_Tick1kHz(bool controller_idle)
{
    if (!s_state.boot_ready) {
        if (s_boot_elapsed_ms < APP_LASER_BOOT_LOCKOUT_MS) {
            s_boot_elapsed_ms++;
        }
        if (s_boot_elapsed_ms >= APP_LASER_BOOT_LOCKOUT_MS) {
            s_state.boot_ready = true;
        }
    }

    if (!s_state.armed) {
        return;
    }

    if (s_state.task_started && controller_idle) {
        s_idle_elapsed_ms++;
        if (s_idle_elapsed_ms >= APP_LASER_IDLE_DISARM_MS) {
            LaserControl_Disarm();
        }
    } else {
        s_idle_elapsed_ms = 0u;
    }
}

void LaserControl_GetState(LaserControlState *out)
{
    if (out != 0) {
        *out = s_state;
    }
}
