#include "home_controller.h"

/* 回零状态机：保留给后续接限位开关使用，课程通信测试可暂时不依赖它。 */

#include "app_config.h"
#include "app_params.h"
#include "home_sensor.h"
#include "stepper_driver.h"

#define HOME_MRAD_PER_REV_I 6283LL

typedef enum {
    HOME_ERR_NONE = 0,
    HOME_ERR_BUSY = 1,
    HOME_ERR_TIMEOUT = 2,
    HOME_ERR_MOVE = 3
} HomeError;

static HomeControllerState s_state;
static uint32_t s_elapsed_ms;
static uint16_t s_debounce_ms;
static uint8_t s_error;
static uint8_t s_move_started;
static bool s_simulated;
static int32_t s_saved_accel_pps_s[2];
static bool s_accel_saved;
static int64_t s_latched_switch_pulse[2];
static bool s_switch_latched[2];

static int32_t axis_search_pps(uint32_t axis)
{
    // Axis1 左红臂：正方向应对应角度增大。若红臂从 90° 向 180° 左水平运动，
    // APP_HOME_AXIS1_DIR 应配置为 +1。
    //
    // Axis2 右蓝臂：若从 90° 向 0° 右水平运动，
    // APP_HOME_AXIS2_DIR 应配置为 -1。
    int32_t dir = axis == 0u ? APP_HOME_AXIS1_DIR : APP_HOME_AXIS2_DIR;
    return dir >= 0 ? APP_HOME_SEARCH_PPS : -APP_HOME_SEARCH_PPS;
}

static int64_t axis_reference_pulse(uint32_t axis, int32_t mrad)
{
    const AppParams *p = AppParams_Get();
    int64_t scale = (int64_t)p->pulses_per_rev[axis] * (int64_t)p->reducer_ratio[axis];
    int64_t delta = (int64_t)mrad - (int64_t)p->motor_zero_mrad[axis];
    return (delta * (int64_t)p->motor_dir_sign[axis] * scale) / HOME_MRAD_PER_REV_I;
}

static int64_t axis_horizontal_pulse(uint32_t axis)
{
    return axis_reference_pulse(axis, axis == 0u ? APP_HOME_AXIS1_HORIZONTAL_MRAD : APP_HOME_AXIS2_HORIZONTAL_MRAD);
}
static int64_t axis_switch_pulse(uint32_t axis)
{
    return axis_reference_pulse(axis, axis == 0u ? APP_HOME_AXIS1_SWITCH_MRAD : APP_HOME_AXIS2_SWITCH_MRAD);
}
static int64_t axis_vertical_pulse(uint32_t axis)
{
    return axis_reference_pulse(axis, axis == 0u ? APP_HOME_AXIS1_VERTICAL_MRAD : APP_HOME_AXIS2_VERTICAL_MRAD);
}

static void enter_state(HomeControllerState state)
{
    s_state = state;
    s_elapsed_ms = 0;
    s_debounce_ms = 0;
    s_move_started = 0;
}

static bool sensor_active(uint32_t axis)
{
    HomeSensorState state;
    HomeSensor_GetState(&state);
    return axis == 0u ? state.home1_active : state.home2_active;
}

static bool debounce_axis(uint32_t axis)
{
    if (sensor_active(axis)) {
        if (s_debounce_ms >= APP_HOME_DEBOUNCE_MS) {
            return true;
        }
    } else {
        s_debounce_ms = 0;
    }
    return false;
}

static bool start_axis_move_to(uint32_t axis, int64_t target)
{
    StepperState stepper;
    Stepper_GetStateSnapshot(&stepper);
    int64_t p1 = stepper.axis[0].position_pulse;
    int64_t p2 = stepper.axis[1].position_pulse;
    if (axis == 0u) {
        p1 = target;
    } else {
        p2 = target;
    }
    return Stepper_MoveAbs(p1, p2, APP_HOME_SEARCH_PPS, APP_HOME_SEARCH_PPS);
}

static void save_motion_accel(void)
{
    StepperState stepper;
    Stepper_GetStateSnapshot(&stepper);
    s_saved_accel_pps_s[0] = stepper.axis[0].accel_pps_s;
    s_saved_accel_pps_s[1] = stepper.axis[1].accel_pps_s;
    s_accel_saved = true;
}

static void restore_motion_accel(void)
{
    if (!s_accel_saved) {
        return;
    }
    Stepper_SetAccel(STEPPER_AXIS_1, s_saved_accel_pps_s[0]);
    Stepper_SetAccel(STEPPER_AXIS_2, s_saved_accel_pps_s[1]);
    s_accel_saved = false;
}

static void latch_switch_reference(uint32_t axis)
{
    if (axis >= 2u) {
        return;
    }
    s_latched_switch_pulse[axis] = axis_switch_pulse(axis);
    s_switch_latched[axis] = true;
}

static void apply_latched_switch_reference(uint32_t axis)
{
    if (axis >= 2u || !s_switch_latched[axis]) {
        return;
    }
    Stepper_SetPosition(axis == 0u ? STEPPER_AXIS_1 : STEPPER_AXIS_2, s_latched_switch_pulse[axis]);
    s_switch_latched[axis] = false;
}

void HomeController_Init(void)
{
    s_state = HOME_CTRL_IDLE;
    s_elapsed_ms = 0;
    s_debounce_ms = 0;
    s_error = HOME_ERR_NONE;
    s_move_started = 0;
    s_simulated = false;
    s_accel_saved = false;
    s_switch_latched[0] = false;
    s_switch_latched[1] = false;
}

bool HomeController_Start(bool simulated)
{
    if (s_state == HOME_CTRL_DONE || s_state == HOME_CTRL_ERROR) {
        enter_state(HOME_CTRL_IDLE);
        s_error = HOME_ERR_NONE;
    }

    if (Stepper_IsBusy() || s_state == HOME_CTRL_AXIS1_SEARCH || s_state == HOME_CTRL_AXIS1_RETURN ||
        s_state == HOME_CTRL_AXIS2_SEARCH || s_state == HOME_CTRL_AXIS2_RETURN) {
        s_error = HOME_ERR_BUSY;
        return false;
    }
    Stepper_ClearError();
    Stepper_EnableAll(true);
    save_motion_accel();

    /* 回零搜索和触发后减速停止使用专用加速度 */
    Stepper_SetAccel(STEPPER_AXIS_1, APP_HOME_ACCEL_PPS_S);
    Stepper_SetAccel(STEPPER_AXIS_2, APP_HOME_ACCEL_PPS_S);
    
    s_simulated = simulated;
    s_switch_latched[0] = false;
    s_switch_latched[1] = false;
    if (!s_simulated) {
        Stepper_SetPps(STEPPER_AXIS_1, axis_search_pps(0));
    }
    s_error = HOME_ERR_NONE;
    enter_state(HOME_CTRL_AXIS1_SEARCH);
    return true;
}

void HomeController_Stop(void)
{
    Stepper_StopAll();
    restore_motion_accel();
    if (s_state != HOME_CTRL_IDLE && s_state != HOME_CTRL_DONE) {
        enter_state(HOME_CTRL_ERROR);
        s_error = HOME_ERR_MOVE;
    }
}

void HomeController_ClearError(void)
{
    if (s_state == HOME_CTRL_ERROR || s_state == HOME_CTRL_DONE) {
        enter_state(HOME_CTRL_IDLE);
    }
    s_error = HOME_ERR_NONE;
}

void HomeController_Loop(void)
{
    if (s_state == HOME_CTRL_IDLE || s_state == HOME_CTRL_DONE || s_state == HOME_CTRL_ERROR) {
        return;
    }

    if (s_elapsed_ms > APP_HOME_TIMEOUT_MS) {
        Stepper_StopAll();
        restore_motion_accel();
        enter_state(HOME_CTRL_ERROR);
        s_error = HOME_ERR_TIMEOUT;
        return;
    }

    if (s_state == HOME_CTRL_AXIS1_SEARCH) {
        if (s_simulated && !s_move_started && Stepper_CanAcceptMove()) {
            if (!start_axis_move_to(0, axis_horizontal_pulse(0))) {
                restore_motion_accel();
                enter_state(HOME_CTRL_ERROR);
                s_error = HOME_ERR_MOVE;
                return;
            }
            s_move_started = 1;
        } else if ((s_simulated && s_move_started && Stepper_CanAcceptMove()) ||
                (!s_simulated && debounce_axis(0))) {
            if (!s_simulated) {
                latch_switch_reference(0);
            }
            Stepper_Stop(STEPPER_AXIS_1);
            enter_state(HOME_CTRL_AXIS1_RETURN);
        }
    } else if (s_state == HOME_CTRL_AXIS1_RETURN) {
        if (!s_move_started && Stepper_CanAcceptMove()) {
            if (!s_simulated) {
                apply_latched_switch_reference(0);
            }
            if (!start_axis_move_to(0, axis_vertical_pulse(0))) {
                restore_motion_accel();
                enter_state(HOME_CTRL_ERROR);
                s_error = HOME_ERR_MOVE;
                return;
            }
            s_move_started = 1;
        } else if (s_move_started && Stepper_CanAcceptMove()) {
            if (s_simulated) {
                enter_state(HOME_CTRL_AXIS2_SEARCH);
            } else {
                Stepper_SetPps(STEPPER_AXIS_2, axis_search_pps(1));
                enter_state(HOME_CTRL_AXIS2_SEARCH);
            }
        }
    } else if (s_state == HOME_CTRL_AXIS2_SEARCH) {
        if (s_simulated && !s_move_started && Stepper_CanAcceptMove()) {
            if (!start_axis_move_to(1, axis_horizontal_pulse(1))) {
                restore_motion_accel();
                enter_state(HOME_CTRL_ERROR);
                s_error = HOME_ERR_MOVE;
                return;
            }
            s_move_started = 1;
        } else if ((s_simulated && s_move_started && Stepper_CanAcceptMove()) ||
                (!s_simulated && debounce_axis(1))) {
            if (!s_simulated) {
                latch_switch_reference(1);
            }
            Stepper_Stop(STEPPER_AXIS_2);
            enter_state(HOME_CTRL_AXIS2_RETURN);
        }
    } else if (s_state == HOME_CTRL_AXIS2_RETURN) {
        if (!s_move_started && Stepper_CanAcceptMove()) {
            if (!s_simulated) {
                apply_latched_switch_reference(1);
            }
            if (!start_axis_move_to(1, axis_vertical_pulse(1))) {
                restore_motion_accel();
                enter_state(HOME_CTRL_ERROR);
                s_error = HOME_ERR_MOVE;
                return;
            }
            s_move_started = 1;
        } else if (s_move_started && Stepper_CanAcceptMove()) {
            restore_motion_accel();
            enter_state(HOME_CTRL_DONE);
        }
    }
}

void HomeController_Tick1kHz(void)
{
    if (s_state != HOME_CTRL_IDLE && s_state != HOME_CTRL_DONE && s_state != HOME_CTRL_ERROR) {
        s_elapsed_ms++;
        if (s_debounce_ms < 0xFFFFu) {
            s_debounce_ms++;
        }
    }
}

HomeControllerState HomeController_GetState(void)
{
    return s_state;
}

uint8_t HomeController_Error(void)
{
    return s_error;
}

const char *HomeController_StateName(HomeControllerState state)
{
    switch (state) {
    case HOME_CTRL_IDLE: return "Idle";
    case HOME_CTRL_AXIS1_SEARCH: return "Axis1Search";
    case HOME_CTRL_AXIS1_RETURN: return "Axis1Return";
    case HOME_CTRL_AXIS2_SEARCH: return "Axis2Search";
    case HOME_CTRL_AXIS2_RETURN: return "Axis2Return";
    case HOME_CTRL_DONE: return "Done";
    case HOME_CTRL_ERROR: return "Error";
    default: return "Unknown";
    }
}
