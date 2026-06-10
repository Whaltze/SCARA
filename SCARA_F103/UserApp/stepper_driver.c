#include "stepper_driver.h"

/* 步进底层：将规划好的目标脉冲和 pps 转成 PWM 输出，并维护软件脉冲计数。 */

#include "app_config.h"
#include "app_params.h"
#include "board_pins.h"

#include <stdlib.h>
#include <string.h>

typedef struct {
    TIM_HandleTypeDef *tim;
    uint32_t channel;
    GPIO_TypeDef *step_port;
    uint16_t step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *ena_port;
    uint16_t ena_pin;
    int32_t pulse_accum;
    int32_t accel_accum;
    int32_t applied_pps;
} StepperHw;

typedef struct {
    bool active;
    bool host_timed;
    uint32_t step_event_count;
    uint32_t remaining_events;
    uint32_t steps[2];
    uint32_t counter[2];
    int8_t dir[2];
    int32_t current_pps;
    int32_t target_pps;
    int32_t exit_pps;
    int32_t accel_pps_s;
    int32_t accel_accum;
    int64_t event_accum;
    uint32_t duration_ticks;
    uint32_t elapsed_ticks;
} StepperDdaMove;

typedef struct {
    int64_t pos1;
    int64_t pos2;
    uint32_t duration_ticks;
} StepperTimedSegment;

static StepperState s_state;
static StepperHw s_hw[2] = {
    {BOARD_M1_TIM, BOARD_M1_TIM_CHANNEL, BOARD_M1_STEP_PORT, BOARD_M1_STEP_PIN, BOARD_M1_DIR_PORT, BOARD_M1_DIR_PIN, BOARD_M1_ENA_PORT, BOARD_M1_ENA_PIN, 0, 0, 0},
    {BOARD_M2_TIM, BOARD_M2_TIM_CHANNEL, BOARD_M2_STEP_PORT, BOARD_M2_STEP_PIN, BOARD_M2_DIR_PORT, BOARD_M2_DIR_PIN, BOARD_M2_ENA_PORT, BOARD_M2_ENA_PIN, 0, 0, 0},
};
static StepperDdaMove s_move;
static StepperTimedSegment s_timed_segments[APP_STEPPER_TIMED_SEGMENTS];
static uint8_t s_timed_head;
static uint8_t s_timed_tail;
static uint8_t s_timed_count;

static bool target_in_range(int64_t target);
static int64_t initial_pulse(uint32_t index);

static uint8_t timed_next_index(uint8_t index)
{
    index++;
    return index >= APP_STEPPER_TIMED_SEGMENTS ? 0u : index;
}

static void timed_queue_clear(void)
{
    s_timed_head = 0u;
    s_timed_tail = 0u;
    s_timed_count = 0u;
}

static bool timed_queue_pop(StepperTimedSegment *segment)
{
    if (s_timed_count == 0u || segment == NULL) {
        return false;
    }
    *segment = s_timed_segments[s_timed_tail];
    s_timed_tail = timed_next_index(s_timed_tail);
    s_timed_count--;
    return true;
}

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int64_t i64_abs(int64_t value)
{
    return value < 0 ? -value : value;
}

static bool axis_valid(StepperAxis axis)
{
    return axis == STEPPER_AXIS_1 || axis == STEPPER_AXIS_2;
}

static GPIO_PinState ena_pin_state(bool enable)
{
#if APP_STEPPER_ENA_ACTIVE_LEVEL
    return enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
#else
    return enable ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif
}

static GPIO_PinState step_pin_state(bool active)
{
#if APP_STEPPER_PUL_ACTIVE_LEVEL
    return active ? GPIO_PIN_SET : GPIO_PIN_RESET;
#else
    return active ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif
}

static GPIO_PinState dir_pin_state(uint32_t index, int8_t dir)
{
    bool positive = dir > 0;
    bool positive_level =
        index == 0u ? (APP_STEPPER_M1_DIR_POSITIVE_LEVEL != 0u) :
                      (APP_STEPPER_M2_DIR_POSITIVE_LEVEL != 0u);
    return positive == positive_level ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static void pulse_width_delay(void)
{
    for (volatile uint32_t i = 0; i < APP_STEPPER_PULSE_WIDTH_NOP; ++i) {
        __NOP();
    }
}

static void dir_setup_delay(void)
{
    for (volatile uint32_t i = 0; i < APP_STEPPER_DIR_SETUP_NOP; ++i) {
        __NOP();
    }
}

static uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void irq_restore(uint32_t primask)
{
    if (primask == 0u) {
        __enable_irq();
    }
}

static void pwm_apply(uint32_t index, int32_t pps)
{
    /* pps 低于最小有效值或轴未使能时关闭 PWM，避免空闲状态继续输出脉冲。 */
    int32_t abs_pps = abs(pps);

    if (abs_pps < APP_MIN_EFFECTIVE_PPS || !s_state.axis[index].enabled) {
        s_hw[index].applied_pps = 0;
        s_state.axis[index].running = false;
        return;
    }

    abs_pps = clamp_i32(abs_pps, APP_MIN_EFFECTIVE_PPS, s_state.axis[index].max_pps);
    pps = pps < 0 ? -abs_pps : abs_pps;
    if (s_hw[index].applied_pps == pps) {
        s_state.axis[index].running = true;
        return;
    }
    s_hw[index].applied_pps = pps;
    s_state.axis[index].running = true;
}

static void emit_step(uint32_t index)
{
    HAL_GPIO_WritePin(s_hw[index].step_port, s_hw[index].step_pin, step_pin_state(true));
    pulse_width_delay();
    HAL_GPIO_WritePin(s_hw[index].step_port, s_hw[index].step_pin, step_pin_state(false));
}

static void set_dir_index(uint32_t index, int8_t dir)
{
    if (dir == 0) {
        return;
    }
    GPIO_PinState old_state = dir_pin_state(index, s_state.axis[index].dir);
    s_state.axis[index].dir = dir > 0 ? 1 : -1;
    GPIO_PinState new_state = dir_pin_state(index, s_state.axis[index].dir);
    HAL_GPIO_WritePin(s_hw[index].dir_port, s_hw[index].dir_pin, new_state);
    if (old_state != new_state) {
        dir_setup_delay();
    }
}

static void axis_set_target_pps(uint32_t index, int32_t pps)
{
    StepperAxisState *axis = &s_state.axis[index];

    if (pps == 0) {
        axis->target_pps = 0;
        return;
    }

    pps = clamp_i32(pps, -axis->max_pps, axis->max_pps);
    set_dir_index(index, pps > 0 ? 1 : -1);
    axis->target_pps = pps;
}

static void axis_stop_now(uint32_t index)
{
    StepperAxisState *axis = &s_state.axis[index];
    s_move.active = false;
    axis->current_pps = 0;
    axis->target_pps = 0;
    axis->exit_pps = 0;
    axis->remaining_pulse = 0;
    axis->mode = STEPPER_MODE_IDLE;
    pwm_apply(index, 0);
}

static int32_t blended_move_accel_pps_s(const StepperState *snapshot)
{
    int32_t accel1 = snapshot->axis[0].accel_pps_s;
    int32_t accel2 = snapshot->axis[1].accel_pps_s;
    int32_t accel = accel1 > accel2 ? accel1 : accel2;
    return clamp_i32(accel, 1, APP_ACCEL_MAX);
}

void Stepper_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    timed_queue_clear();
    for (uint32_t i = 0; i < 2u; ++i) {
        // 上电直接使能
        s_state.axis[i].enabled = APP_STEPPER_ENABLE_ON_BOOT != 0u;
        s_state.axis[i].running = false;
        s_state.axis[i].dir = 1;
        s_state.axis[i].mode = STEPPER_MODE_IDLE;
        s_state.axis[i].current_pps = 0;
        s_state.axis[i].target_pps = 0;
        s_state.axis[i].exit_pps = 0;
        s_state.axis[i].accel_pps_s = APP_ACCEL_DEFAULT;
        s_state.axis[i].max_pps = APP_MAX_PPS_DEFAULT;
        int64_t init_pulse = initial_pulse(i);
        s_state.axis[i].position_pulse = init_pulse;
        s_state.axis[i].target_position_pulse = init_pulse;
        s_state.axis[i].remaining_pulse = 0;
        s_state.axis[i].error = 0;
        s_hw[i].pulse_accum = 0;
        s_hw[i].accel_accum = 0;
        s_hw[i].applied_pps = 0;
        gpio.Pin = s_hw[i].dir_pin;
        gpio.Mode = GPIO_MODE_OUTPUT_OD;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(s_hw[i].dir_port, &gpio);
        HAL_GPIO_WritePin(s_hw[i].dir_port, s_hw[i].dir_pin, dir_pin_state(i, s_state.axis[i].dir));
        s_hw[i].step_port->BSRR = s_hw[i].step_pin;
        gpio.Pin = s_hw[i].step_pin;
        gpio.Mode = GPIO_MODE_OUTPUT_OD;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(s_hw[i].step_port, &gpio);
        gpio.Pin = s_hw[i].ena_pin;
        gpio.Mode = GPIO_MODE_OUTPUT_OD;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(s_hw[i].ena_port, &gpio);
        HAL_GPIO_WritePin(s_hw[i].ena_port, s_hw[i].ena_pin,
                          ena_pin_state(s_state.axis[i].enabled));
    }
    memset(&s_move, 0, sizeof(s_move));
}

void Stepper_Enable(StepperAxis axis, bool enable)
{
    if (!axis_valid(axis)) {
        return;
    }

    uint32_t i = (uint32_t)axis;
    s_state.axis[i].enabled = enable;
    HAL_GPIO_WritePin(s_hw[i].ena_port, s_hw[i].ena_pin, ena_pin_state(enable));
    if (!enable) {
        uint32_t primask = irq_save();
        timed_queue_clear();
        axis_stop_now(i);
        irq_restore(primask);
    }
}

void Stepper_EnableAll(bool enable)
{
    Stepper_Enable(STEPPER_AXIS_1, enable);
    Stepper_Enable(STEPPER_AXIS_2, enable);
}

void Stepper_SetDir(StepperAxis axis, int8_t dir)
{
    if (axis_valid(axis)) {
        set_dir_index((uint32_t)axis, dir);
    }
}

void Stepper_SetPps(StepperAxis axis, int32_t pps)
{
    if (!axis_valid(axis)) {
        return;
    }
    uint32_t i = (uint32_t)axis;
    s_state.axis[i].mode = pps == 0 ? STEPPER_MODE_STOPPING : STEPPER_MODE_SPEED;
    axis_set_target_pps(i, pps);
}

void Stepper_SetAccel(StepperAxis axis, int32_t accel_pps_s)
{
    if (!axis_valid(axis)) {
        return;
    }
    s_state.axis[(uint32_t)axis].accel_pps_s = clamp_i32(accel_pps_s, 1, APP_ACCEL_MAX);
}

static bool target_in_range(int64_t target)
{
#if APP_HOST_OWNS_LIMIT_CHECKS
    (void)target;
    return true;
#else
    const AppParams *p = AppParams_Get();
    return target >= p->motor_min_pos && target <= p->motor_max_pos;
#endif
}

static int64_t reference_pulse(uint32_t index, int32_t theta_mrad)
{
    const AppParams *p = AppParams_Get();
    int64_t scale = (int64_t)p->pulses_per_rev[index] * p->reducer_ratio[index];
    int64_t delta = (int64_t)theta_mrad - p->motor_zero_mrad[index];
    return (delta * p->motor_dir_sign[index] * scale) / 6283LL;
}

static int64_t initial_pulse(uint32_t index)
{
    return reference_pulse(index, index == 0u ? APP_HOME_AXIS1_VERTICAL_MRAD : APP_HOME_AXIS2_VERTICAL_MRAD);
}

static bool __attribute__((unused)) move_prepare(uint32_t index,
                                                 int64_t current_position,
                                                 int64_t target,
                                                 int32_t vmax,
                                                 int32_t exit_pps)
{
    /* 准备单轴移动段：目标位置、剩余脉冲、方向、目标速度和出口速度。 */
    StepperAxisState *axis = &s_state.axis[index];
    int64_t delta = target - current_position;

    if (!target_in_range(target)) {
        axis->error |= STEPPER_ERR_SOFT_LIMIT;
        return false;
    }

    axis->target_position_pulse = target;
    axis->remaining_pulse = i64_abs(delta);
    if (axis->remaining_pulse == 0 || vmax == 0) {
        axis->target_pps = 0;
        axis->exit_pps = 0;
        axis->mode = STEPPER_MODE_IDLE;
        pwm_apply(index, 0);
        axis->target_position_pulse = target;
        return true;
    }

    int8_t dir = delta > 0 ? 1 : -1;
    vmax = abs(vmax);
    vmax = clamp_i32(vmax, APP_MIN_EFFECTIVE_PPS, axis->max_pps);
    set_dir_index(index, dir);
    axis->mode = STEPPER_MODE_MOVE;
    axis->target_pps = dir > 0 ? vmax : -vmax;
    exit_pps = abs(exit_pps);
    if (exit_pps > vmax) {
        exit_pps = vmax;
    }
    axis->exit_pps = dir > 0 ? exit_pps : -exit_pps;
    return true;
}

bool Stepper_MoveRel(int64_t delta1, int64_t delta2, int32_t v1, int32_t v2)
{
    StepperState snapshot;
    Stepper_GetStateSnapshot(&snapshot);
    return Stepper_MoveAbs(snapshot.axis[0].position_pulse + delta1,
                           snapshot.axis[1].position_pulse + delta2,
                           v1, v2);
}

bool Stepper_MoveAbs(int64_t pos1, int64_t pos2, int32_t v1, int32_t v2)
{
    return Stepper_MoveAbsBlend(pos1, pos2, v1, v2, 0, 0);
}

bool Stepper_MoveAbsBlend(int64_t pos1, int64_t pos2, int32_t v1, int32_t v2, int32_t exit1, int32_t exit2)
{
    StepperState snapshot;
    Stepper_GetStateSnapshot(&snapshot);
    int64_t cur1 = snapshot.axis[0].position_pulse;
    int64_t cur2 = snapshot.axis[1].position_pulse;
    int64_t d1 = i64_abs(pos1 - cur1);
    int64_t d2 = i64_abs(pos2 - cur2);
    int32_t av1 = abs(v1);
    int32_t av2 = abs(v2);

    if (snapshot.axis[0].mode == STEPPER_MODE_ESTOP || snapshot.axis[1].mode == STEPPER_MODE_ESTOP) {
        s_state.axis[0].error |= STEPPER_ERR_ESTOP;
        s_state.axis[1].error |= STEPPER_ERR_ESTOP;
        return false;
    }

    if (!target_in_range(pos1) || !target_in_range(pos2)) {
        if (!target_in_range(pos1)) {
            s_state.axis[0].error |= STEPPER_ERR_SOFT_LIMIT;
        }
        if (!target_in_range(pos2)) {
            s_state.axis[1].error |= STEPPER_ERR_SOFT_LIMIT;
        }
        return false;
    }

    if ((d1 > 0 && !snapshot.axis[0].enabled) || (d2 > 0 && !snapshot.axis[1].enabled)) {
        if (d1 > 0 && !snapshot.axis[0].enabled) {
            s_state.axis[0].error |= STEPPER_ERR_DISABLED;
        }
        if (d2 > 0 && !snapshot.axis[1].enabled) {
            s_state.axis[1].error |= STEPPER_ERR_DISABLED;
        }
        return false;
    }

    if (d1 > 0 && av1 == 0) {
        av1 = APP_MAX_PPS_DEFAULT;
    }
    if (d2 > 0 && av2 == 0) {
        av2 = APP_MAX_PPS_DEFAULT;
    }

    if (d1 > 0 && d2 > 0 && av1 > 0 && av2 > 0) {
        uint64_t t1_us = ((uint64_t)d1 * 1000000ull) / (uint32_t)av1;
        uint64_t t2_us = ((uint64_t)d2 * 1000000ull) / (uint32_t)av2;
        uint64_t t_us = t1_us > t2_us ? t1_us : t2_us;
        if (t_us > 0) {
            av1 = (int32_t)(((uint64_t)d1 * 1000000ull) / t_us);
            av2 = (int32_t)(((uint64_t)d2 * 1000000ull) / t_us);
            if (av1 > 0 && av1 < APP_INTERPOLATOR_MIN_PPS) {
                av1 = APP_INTERPOLATOR_MIN_PPS;
            }
            if (av2 > 0 && av2 < APP_INTERPOLATOR_MIN_PPS) {
                av2 = APP_INTERPOLATOR_MIN_PPS;
            }
        }
    }

    uint32_t events = (uint32_t)(d1 > d2 ? d1 : d2);
    if (events == 0u) {
        return true;
    }
    int32_t dom_v = av1 > av2 ? av1 : av2;
    if (dom_v <= 0) {
        dom_v = APP_MAX_PPS_DEFAULT;
    }
    dom_v = clamp_i32(dom_v, APP_INTERPOLATOR_MIN_PPS, APP_MAX_PPS_DEFAULT);

    uint32_t primask = irq_save();
    s_move.active = true;
    s_move.host_timed = false;
    s_move.step_event_count = events;
    s_move.remaining_events = events;
    s_move.steps[0] = (uint32_t)d1;
    s_move.steps[1] = (uint32_t)d2;
    s_move.counter[0] = events >> 1;
    s_move.counter[1] = events >> 1;
    s_move.dir[0] = pos1 >= cur1 ? 1 : -1;
    s_move.dir[1] = pos2 >= cur2 ? 1 : -1;
    s_move.target_pps = dom_v;
    if (s_move.current_pps < APP_INTERPOLATOR_MIN_PPS) {
        s_move.current_pps = APP_INTERPOLATOR_MIN_PPS;
    }
    s_move.exit_pps = abs(exit1) > abs(exit2) ? abs(exit1) : abs(exit2);
    if (s_move.exit_pps > dom_v) {
        s_move.exit_pps = dom_v;
    }
    s_move.accel_pps_s = blended_move_accel_pps_s(&snapshot);
    s_move.accel_accum = 0;
    s_move.event_accum = 0;

    int64_t targets[2] = {pos1, pos2};
    int32_t av[2] = {av1, av2};
    for (uint32_t i = 0; i < 2u; ++i) {
        StepperAxisState *axis = &s_state.axis[i];
        axis->target_position_pulse = targets[i];
        axis->remaining_pulse = i == 0u ? d1 : d2;
        axis->target_pps = s_move.dir[i] > 0 ? av[i] : -av[i];
        axis->exit_pps = 0;
        axis->mode = axis->remaining_pulse > 0 ? STEPPER_MODE_MOVE : STEPPER_MODE_IDLE;
        if (axis->remaining_pulse > 0) {
            set_dir_index(i, s_move.dir[i]);
        }
        pwm_apply(i, axis->target_pps);
    }
    irq_restore(primask);
    return true;
}

static bool __attribute__((optimize("Os"))) timed_move_start_locked(int64_t pos1,
                                                                    int64_t pos2,
                                                                    uint32_t duration_ticks)
{
    if (duration_ticks == 0u) {
        return false;
    }

    int64_t cur1 = s_state.axis[0].position_pulse;
    int64_t cur2 = s_state.axis[1].position_pulse;
    int64_t d1 = i64_abs(pos1 - cur1);
    int64_t d2 = i64_abs(pos2 - cur2);

    if (s_state.axis[0].mode == STEPPER_MODE_ESTOP || s_state.axis[1].mode == STEPPER_MODE_ESTOP) {
        s_state.axis[0].error |= STEPPER_ERR_ESTOP;
        s_state.axis[1].error |= STEPPER_ERR_ESTOP;
        return false;
    }
    if (!Stepper_TargetsAllowed(pos1, pos2)) {
        s_state.axis[0].error |= STEPPER_ERR_SOFT_LIMIT;
        s_state.axis[1].error |= STEPPER_ERR_SOFT_LIMIT;
        return false;
    }
    for (uint32_t i = 0; i < 2u; ++i) {
        if ((i == 0u ? d1 : d2) > 0 && !s_state.axis[i].enabled) {
            s_state.axis[i].error |= STEPPER_ERR_DISABLED;
            return false;
        }
    }

    uint32_t events = (uint32_t)(d1 > d2 ? d1 : d2);
    if (events == 0u) {
        return true;
    }
    if (events > duration_ticks) {
        s_state.axis[0].error |= STEPPER_ERR_INVALID_ARG;
        s_state.axis[1].error |= STEPPER_ERR_INVALID_ARG;
        return false;
    }

    s_move.active = true;
    s_move.host_timed = true;
    s_move.step_event_count = events;
    s_move.remaining_events = events;
    s_move.steps[0] = (uint32_t)d1;
    s_move.steps[1] = (uint32_t)d2;
    s_move.counter[0] = events >> 1;
    s_move.counter[1] = events >> 1;
    s_move.dir[0] = pos1 >= cur1 ? 1 : -1;
    s_move.dir[1] = pos2 >= cur2 ? 1 : -1;
    s_move.current_pps = (int32_t)(((uint64_t)events * APP_CONTROL_HZ + (duration_ticks / 2u)) / duration_ticks);
    if (s_move.current_pps < APP_INTERPOLATOR_MIN_PPS) {
        s_move.current_pps = APP_INTERPOLATOR_MIN_PPS;
    }
    s_move.target_pps = s_move.current_pps;
    s_move.exit_pps = 0;
    s_move.accel_pps_s = 0;
    s_move.accel_accum = 0;
    s_move.event_accum = 0;
    s_move.duration_ticks = duration_ticks;
    s_move.elapsed_ticks = 0;

    int64_t targets[2] = {pos1, pos2};
    for (uint32_t i = 0; i < 2u; ++i) {
        StepperAxisState *axis = &s_state.axis[i];
        axis->target_position_pulse = targets[i];
        axis->remaining_pulse = i == 0u ? d1 : d2;
        int32_t axis_pps = 0;
        if (s_move.steps[i] > 0u) {
            axis_pps = (int32_t)(((uint64_t)s_move.steps[i] * APP_CONTROL_HZ + (duration_ticks / 2u)) / duration_ticks);
            if (axis_pps < APP_INTERPOLATOR_MIN_PPS) {
                axis_pps = APP_INTERPOLATOR_MIN_PPS;
            }
        }
        axis->target_pps = s_move.steps[i] > 0u ? (s_move.dir[i] * axis_pps) : 0;
        axis->current_pps = axis->target_pps;
        axis->exit_pps = 0;
        axis->mode = axis->remaining_pulse > 0 ? STEPPER_MODE_MOVE : STEPPER_MODE_IDLE;
        s_hw[i].pulse_accum = 0;
        s_hw[i].accel_accum = 0;
        if (s_move.steps[i] > 0u) {
            set_dir_index(i, s_move.dir[i]);
        }
        pwm_apply(i, axis->current_pps);
    }
    return true;
}

bool Stepper_MoveAbsTicks(int64_t pos1, int64_t pos2, uint32_t duration_ticks)
{
    if (duration_ticks == 0u) {
        return false;
    }

    uint32_t primask = irq_save();
    if (s_move.active) {
        if (!s_move.host_timed || s_timed_count >= APP_STEPPER_TIMED_SEGMENTS) {
            irq_restore(primask);
            return false;
        }
        uint8_t last_index = s_timed_count > 0u
                                 ? (s_timed_head == 0u ? APP_STEPPER_TIMED_SEGMENTS - 1u : s_timed_head - 1u)
                                 : 0u;
        int64_t base1 = s_timed_count > 0u ? s_timed_segments[last_index].pos1
                                           : s_state.axis[0].target_position_pulse;
        int64_t base2 = s_timed_count > 0u ? s_timed_segments[last_index].pos2
                                           : s_state.axis[1].target_position_pulse;
        uint64_t d1 = (uint64_t)i64_abs(pos1 - base1);
        uint64_t d2 = (uint64_t)i64_abs(pos2 - base2);
        uint64_t events = d1 > d2 ? d1 : d2;
        if (events == 0u) {
            irq_restore(primask);
            return true;
        }
        if (events > duration_ticks || !target_in_range(pos1) || !target_in_range(pos2) ||
            (d1 > 0u && !s_state.axis[0].enabled) || (d2 > 0u && !s_state.axis[1].enabled)) {
            irq_restore(primask);
            return false;
        }
        s_timed_segments[s_timed_head].pos1 = pos1;
        s_timed_segments[s_timed_head].pos2 = pos2;
        s_timed_segments[s_timed_head].duration_ticks = duration_ticks;
        s_timed_head = timed_next_index(s_timed_head);
        s_timed_count++;
        irq_restore(primask);
        return true;
    }

    bool started = timed_move_start_locked(pos1, pos2, duration_ticks);
    irq_restore(primask);
    return started;
}

void Stepper_Stop(StepperAxis axis)
{
    if (!axis_valid(axis)) {
        return;
    }
    StepperAxisState *s = &s_state.axis[(uint32_t)axis];
    if (s->mode != STEPPER_MODE_ESTOP) {
        s->target_pps = 0;
        s->mode = STEPPER_MODE_STOPPING;
    }
}

void Stepper_StopAll(void)
{
    uint32_t primask = irq_save();
    timed_queue_clear();
    s_move.active = false;
    s_move.host_timed = false;
    irq_restore(primask);
    Stepper_Stop(STEPPER_AXIS_1);
    Stepper_Stop(STEPPER_AXIS_2);
}

void Stepper_EStopAll(void)
{
    uint32_t primask = irq_save();
    timed_queue_clear();
    for (uint32_t i = 0; i < 2u; ++i) {
        axis_stop_now(i);
        s_state.axis[i].mode = STEPPER_MODE_ESTOP;
    }
    irq_restore(primask);
}

void Stepper_ClearError(void)
{
    for (uint32_t i = 0; i < 2u; ++i) {
        if (s_state.axis[i].mode == STEPPER_MODE_ESTOP) {
            s_state.axis[i].mode = STEPPER_MODE_IDLE;
        }
        s_state.axis[i].error = 0;
    }
}

void Stepper_SetErrorAll(uint32_t error_bits)
{
    for (uint32_t i = 0; i < 2u; ++i) {
        s_state.axis[i].error |= error_bits;
    }
}

void Stepper_SetPosition(StepperAxis axis, int64_t position_pulse)
{
    if (!axis_valid(axis)) {
        return;
    }

    uint32_t index = (uint32_t)axis;
    uint32_t primask = irq_save();
    s_move.active = false;
    timed_queue_clear();
    s_state.axis[index].position_pulse = position_pulse;
    s_state.axis[index].target_position_pulse = position_pulse;
    s_state.axis[index].remaining_pulse = 0;
    s_hw[index].pulse_accum = 0;
    s_hw[index].accel_accum = 0;
    irq_restore(primask);
}

void Stepper_Zero(void)
{
    uint32_t primask = irq_save();
    s_move.active = false;
    timed_queue_clear();
    for (uint32_t i = 0; i < 2u; ++i) {
        s_state.axis[i].position_pulse = 0;
        s_state.axis[i].target_position_pulse = 0;
        s_state.axis[i].remaining_pulse = 0;
        s_hw[i].pulse_accum = 0;
        s_hw[i].accel_accum = 0;
    }
    irq_restore(primask);
}

static void axis_tick(uint32_t index)
{
    /* 1 kHz 轻量 tick：只做速度斜坡、PWM 更新和软件计脉冲，不解析串口。 */
    StepperAxisState *axis = &s_state.axis[index];
    if (!axis->enabled) {
        axis_stop_now(index);
        s_hw[index].pulse_accum = 0;
        s_hw[index].accel_accum = 0;
        return;
    }

    if (s_move.active && axis->mode == STEPPER_MODE_MOVE) {
        return;
    }

    if (axis->mode == STEPPER_MODE_IDLE) {
        axis->target_pps = 0;
        axis->current_pps = 0;
        s_hw[index].accel_accum = 0;
        pwm_apply(index, 0);
        return;
    }

    if (axis->mode == STEPPER_MODE_MOVE &&
        axis->remaining_pulse > 0 &&
        axis->current_pps == 0 &&
        axis->target_pps == 0) {
        int8_t dir = axis->target_position_pulse >= axis->position_pulse ? 1 : -1;
        axis->target_pps = dir > 0 ? APP_MIN_EFFECTIVE_PPS : -APP_MIN_EFFECTIVE_PPS;
    }

    if (axis->mode == STEPPER_MODE_MOVE && axis->remaining_pulse > 0) {
        int32_t abs_cur = abs(axis->current_pps);
        int32_t abs_exit = abs(axis->exit_pps);
        int64_t stop_dist = 0;
        if (abs_cur > abs_exit) {
            stop_dist = (((int64_t)abs_cur * (int64_t)abs_cur) -
                         ((int64_t)abs_exit * (int64_t)abs_exit)) / (2 * axis->accel_pps_s);
        }
        if (abs_cur > APP_MIN_EFFECTIVE_PPS && axis->remaining_pulse <= stop_dist + 1) {
            axis->target_pps = axis->exit_pps;
        }
    }

    if (axis->current_pps != axis->target_pps) {
        int32_t diff = axis->target_pps - axis->current_pps;
        int32_t sign = diff > 0 ? 1 : -1;
        int32_t steps = 0;

        s_hw[index].accel_accum += axis->accel_pps_s;
        while (s_hw[index].accel_accum >= (int32_t)APP_CONTROL_HZ) {
            s_hw[index].accel_accum -= (int32_t)APP_CONTROL_HZ;
            steps++;
        }
        if (steps > 0) {
            if (steps > abs(diff)) {
                steps = abs(diff);
            }
            axis->current_pps += sign * steps;
        }
    } else {
        s_hw[index].accel_accum = 0;
    }

    pwm_apply(index, axis->current_pps);

    int32_t abs_pps = abs(axis->current_pps);
    s_hw[index].pulse_accum += abs_pps;
    while (s_hw[index].pulse_accum >= (int32_t)APP_CONTROL_HZ) {
        s_hw[index].pulse_accum -= (int32_t)APP_CONTROL_HZ;
        int8_t step_dir = axis->current_pps >= 0 ? 1 : -1;
        emit_step(index);
        axis->position_pulse += step_dir;
        if (axis->mode == STEPPER_MODE_MOVE && axis->remaining_pulse > 0) {
            axis->remaining_pulse--;
            if (axis->remaining_pulse == 0) {
                axis->position_pulse = axis->target_position_pulse;
                axis->mode = STEPPER_MODE_IDLE;
                axis->target_pps = 0;
                axis->exit_pps = 0;
                pwm_apply(index, 0);
                break;
            }
        }
    }

    if (axis->mode == STEPPER_MODE_STOPPING && axis->current_pps == 0) {
        axis->mode = STEPPER_MODE_IDLE;
    }
}

static void __attribute__((optimize("Os"))) dda_tick(void)
{
    if (!s_move.active) {
        return;
    }
    if (!s_state.axis[0].enabled && s_move.steps[0] > 0u) {
        axis_stop_now(0);
        s_move.active = false;
        return;
    }
    if (!s_state.axis[1].enabled && s_move.steps[1] > 0u) {
        axis_stop_now(1);
        s_move.active = false;
        return;
    }

    if (s_move.host_timed) {
        s_move.elapsed_ticks++;
        s_move.event_accum += s_move.step_event_count;
        while (s_move.event_accum >= (int64_t)s_move.duration_ticks && s_move.remaining_events > 0u) {
            s_move.event_accum -= (int64_t)s_move.duration_ticks;
            for (uint32_t i = 0; i < 2u; ++i) {
                if (s_move.steps[i] == 0u) {
                    continue;
                }
                s_move.counter[i] += s_move.steps[i];
                if (s_move.counter[i] >= s_move.step_event_count) {
                    s_move.counter[i] -= s_move.step_event_count;
                    emit_step(i);
                    s_state.axis[i].position_pulse += s_move.dir[i];
                    if (s_state.axis[i].remaining_pulse > 0) {
                        s_state.axis[i].remaining_pulse--;
                    }
                }
            }
            s_move.remaining_events--;
        }

        if (s_move.elapsed_ticks >= s_move.duration_ticks || s_move.remaining_events == 0u) {
            for (uint32_t i = 0; i < 2u; ++i) {
                s_state.axis[i].position_pulse = s_state.axis[i].target_position_pulse;
                s_state.axis[i].remaining_pulse = 0;
                s_state.axis[i].mode = STEPPER_MODE_IDLE;
                s_state.axis[i].target_pps = 0;
                s_state.axis[i].current_pps = 0;
                s_state.axis[i].exit_pps = 0;
                pwm_apply(i, 0);
            }
            s_move.active = false;
            s_move.host_timed = false;
            s_move.current_pps = 0;
            s_move.target_pps = 0;
            StepperTimedSegment next;
            if (timed_queue_pop(&next)) {
                (void)timed_move_start_locked(next.pos1, next.pos2, next.duration_ticks);
            }
        }
        return;
    }

    if (s_move.current_pps != s_move.target_pps) {
        int32_t diff = s_move.target_pps - s_move.current_pps;
        int32_t sign = diff > 0 ? 1 : -1;
        int32_t steps = 0;
        s_move.accel_accum += s_move.accel_pps_s;
        while (s_move.accel_accum >= (int32_t)APP_CONTROL_HZ) {
            s_move.accel_accum -= (int32_t)APP_CONTROL_HZ;
            steps++;
        }
        if (steps > 0) {
            if (steps > abs(diff)) {
                steps = abs(diff);
            }
            s_move.current_pps += sign * steps;
        }
    } else {
        s_move.accel_accum = 0;
    }

    if (s_move.remaining_events > 0u && s_move.current_pps < APP_INTERPOLATOR_MIN_PPS) {
        s_move.current_pps = APP_INTERPOLATOR_MIN_PPS;
    }

    int64_t stop_dist = 0;
    if (s_move.current_pps > s_move.exit_pps) {
        stop_dist = (((int64_t)s_move.current_pps * s_move.current_pps) -
                     ((int64_t)s_move.exit_pps * s_move.exit_pps)) / (2 * s_move.accel_pps_s);
    }
    if (s_move.current_pps > APP_INTERPOLATOR_MIN_PPS && s_move.remaining_events <= (uint32_t)(stop_dist + 1)) {
        s_move.target_pps = s_move.exit_pps > APP_INTERPOLATOR_MIN_PPS ? s_move.exit_pps : APP_INTERPOLATOR_MIN_PPS;
    }

    for (uint32_t i = 0; i < 2u; ++i) {
        s_state.axis[i].current_pps = s_move.steps[i] > 0u ? (s_move.dir[i] * s_move.current_pps) : 0;
        pwm_apply(i, s_state.axis[i].current_pps);
    }

    s_move.event_accum += s_move.current_pps;
    while (s_move.event_accum >= (int32_t)APP_CONTROL_HZ && s_move.remaining_events > 0u) {
        s_move.event_accum -= (int32_t)APP_CONTROL_HZ;
        for (uint32_t i = 0; i < 2u; ++i) {
            if (s_move.steps[i] == 0u) {
                continue;
            }
            s_move.counter[i] += s_move.steps[i];
            if (s_move.counter[i] >= s_move.step_event_count) {
                s_move.counter[i] -= s_move.step_event_count;
                emit_step(i);
                s_state.axis[i].position_pulse += s_move.dir[i];
                if (s_state.axis[i].remaining_pulse > 0) {
                    s_state.axis[i].remaining_pulse--;
                }
            }
        }
        s_move.remaining_events--;
    }

    if (s_move.remaining_events == 0u) {
        int32_t completed_exit_pps = s_move.exit_pps;
        for (uint32_t i = 0; i < 2u; ++i) {
            s_state.axis[i].position_pulse = s_state.axis[i].target_position_pulse;
            s_state.axis[i].remaining_pulse = 0;
            s_state.axis[i].mode = STEPPER_MODE_IDLE;
            s_state.axis[i].target_pps = 0;
            s_state.axis[i].current_pps = 0;
            s_state.axis[i].exit_pps = 0;
            pwm_apply(i, 0);
        }
        s_move.active = false;
        s_move.current_pps = completed_exit_pps;
        s_move.target_pps = 0;
    }
}

void Stepper_Tick10kHz(void)
{
    static uint8_t tick_divider;
    tick_divider++;
    if (tick_divider >= 10u) {
        tick_divider = 0;
        s_state.tick_ms++;
    }
    dda_tick();
    axis_tick(0);
    axis_tick(1);
}

bool Stepper_IsBusy(void)
{
    if (s_move.active || s_timed_count > 0u) {
        return true;
    }
    for (uint32_t i = 0; i < 2u; ++i) {
        if (s_state.axis[i].mode == STEPPER_MODE_SPEED ||
            s_state.axis[i].mode == STEPPER_MODE_MOVE ||
            s_state.axis[i].mode == STEPPER_MODE_STOPPING ||
            s_state.axis[i].running) {
            return true;
        }
    }
    return false;
}

bool Stepper_CanAcceptMove(void)
{
    if (s_move.active) {
        return false;
    }
    for (uint32_t i = 0; i < 2u; ++i) {
        if (s_state.axis[i].mode == STEPPER_MODE_SPEED ||
            s_state.axis[i].mode == STEPPER_MODE_MOVE ||
            s_state.axis[i].mode == STEPPER_MODE_STOPPING ||
            s_state.axis[i].mode == STEPPER_MODE_ESTOP) {
            return false;
        }
    }
    return true;
}

bool Stepper_CanQueueTimedSegment(void)
{
    if (s_move.active) {
        return s_move.host_timed && s_timed_count < APP_STEPPER_TIMED_SEGMENTS;
    }
    return Stepper_CanAcceptMove();
}

uint8_t Stepper_TimedSegmentCount(void)
{
    return s_timed_count;
}

uint8_t Stepper_TimedSegmentFree(void)
{
    return (uint8_t)(APP_STEPPER_TIMED_SEGMENTS - s_timed_count);
}

bool Stepper_TargetsAllowed(int64_t pos1, int64_t pos2)
{
    return target_in_range(pos1) && target_in_range(pos2);
}

const StepperState *Stepper_GetState(void)
{
    return &s_state;
}

void Stepper_GetStateSnapshot(StepperState *out)
{
    if (out == NULL) {
        return;
    }

    uint32_t primask = irq_save();
    memcpy(out, &s_state, sizeof(*out));
    irq_restore(primask);
}

const char *Stepper_ModeName(StepperMode mode)
{
    switch (mode) {
    case STEPPER_MODE_IDLE: return "IDLE";
    case STEPPER_MODE_SPEED: return "SPEED";
    case STEPPER_MODE_MOVE: return "MOVE";
    case STEPPER_MODE_STOPPING: return "STOPPING";
    case STEPPER_MODE_ESTOP: return "ESTOP";
    default: return "UNKNOWN";
    }
}
