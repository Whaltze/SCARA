#include "gcode_stream.h"

/*
 * Grbl-style streaming G-code front end for the SCARA Cartesian planner.
 * Lines are acknowledged when parsed and accepted into the planner. Realtime
 * commands bypass this parser in serial_dma.c.
 */

#include "app_config.h"
#include "app_params.h"
#include "home_controller.h"
#include "home_sensor.h"
#include "laser_control.h"
#include "motion_planner.h"
#include "scara_kinematics.h"
#include "serial_dma.h"
#include "stepper_driver.h"

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define GC_PI_F 3.14159265358979323846f

typedef struct {
    uint8_t absolute;
    uint8_t mm_units;
    uint8_t motion_mode;
    uint8_t laser_mode;
    int32_t feed_mm_min;
    int32_t x_um;
    int32_t y_um;
    int32_t offset_x_um;
    int32_t offset_y_um;
    uint16_t spindle_speed;
} GcodeState;

static GcodeState s_gc;
static volatile uint8_t s_status_due;
static uint16_t s_status_divider;
static uint8_t s_hold;
static uint8_t s_home_pending_ack;
static uint8_t s_jog_pending_ack;
static uint8_t s_motion_settle_pending;

static int32_t i32_abs(int32_t value)
{
    return value < 0 ? -value : value;
}

static bool starts_gcode(const char *line)
{
    while (*line != '\0' && isspace((unsigned char)*line)) {
        line++;
    }
    char ch = (char)toupper((unsigned char)*line);
    return ch == 'G' || ch == 'M' || ch == '$' || ch == '?' || ch == '!' ||
           ch == '~' || (unsigned char)ch == 0x18u || (unsigned char)ch == 0x85u;
}

static bool is_home_sim_command(const char *line)
{
    const char *cursor = line + 2;
    while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
        cursor++;
    }
    return toupper((unsigned char)*cursor) == 'S';
}

static bool parse_int_word(const char **cursor, int32_t *out)
{
    int32_t value = 0;
    bool digit = false;
    while (**cursor >= '0' && **cursor <= '9') {
        digit = true;
        value = value * 10 + (**cursor - '0');
        (*cursor)++;
    }
    if (!digit) {
        return false;
    }
    *out = value;
    return true;
}

static bool parse_decimal_scaled(const char **cursor, int32_t scale, int32_t *out)
{
    int32_t sign = 1;
    int64_t whole = 0;
    int64_t fraction = 0;
    int32_t fraction_scale = scale;
    bool digit = false;
    if (**cursor == '+' || **cursor == '-') {
        sign = **cursor == '-' ? -1 : 1;
        (*cursor)++;
    }
    while (**cursor >= '0' && **cursor <= '9') {
        digit = true;
        whole = whole * 10 + (**cursor - '0');
        (*cursor)++;
    }
    if (**cursor == '.') {
        (*cursor)++;
        while (**cursor >= '0' && **cursor <= '9') {
            digit = true;
            if (fraction_scale > 1) {
                fraction_scale /= 10;
                fraction += (int64_t)(**cursor - '0') * fraction_scale;
            }
            (*cursor)++;
        }
    }
    if (!digit) {
        return false;
    }
    *out = (int32_t)((whole * scale + fraction) * sign);
    return true;
}

static void send_error(uint8_t code)
{
    SerialDma_SendFormat("error:%u\n", (unsigned int)code);
}

static void send_ok(void)
{
    SerialDma_Send("ok\n");
}

static void sync_position_from_stepper(void)
{
    StepperState state;
    ScaraPose pose;
    Stepper_GetStateSnapshot(&state);
    if (ScaraKinematics_PulseToPose(state.axis[0].position_pulse, state.axis[1].position_pulse, &pose)) {
        s_gc.x_um = pose.x_um;
        s_gc.y_um = pose.y_um;
    }
}

static void resync_parser_to_stepper_if_diverged(void)
{
    /*
     * The parser position is commanded Cartesian micrometres; the machine truth is
     * integer joint pulses reached through nonlinear IK. A cleanly finished move
     * leaves the stepper at exactly round(IK(commanded endpoint)), so the divergence
     * is zero and the exact commanded coordinate is kept (no FK/IK round-trip noise).
     * An interrupted or fault-stopped move (e.g. preparation_fault, soft-limit clamp,
     * cancel) leaves the parser ahead of where the arm actually stopped; re-anchor to
     * the stepped pulses so the next relative/jog move starts from the true position.
     */
    int64_t cmd_p1;
    int64_t cmd_p2;
    if (!ScaraKinematics_InverseUmToPulse(s_gc.x_um, s_gc.y_um, &cmd_p1, &cmd_p2)) {
        sync_position_from_stepper();
        return;
    }
    StepperState state;
    Stepper_GetStateSnapshot(&state);
    int64_t d1 = state.axis[0].position_pulse - cmd_p1;
    int64_t d2 = state.axis[1].position_pulse - cmd_p2;
    if (d1 < 0) {
        d1 = -d1;
    }
    if (d2 < 0) {
        d2 = -d2;
    }
    if (d1 > APP_POSITION_RESYNC_PULSE_TOL || d2 > APP_POSITION_RESYNC_PULSE_TOL) {
        sync_position_from_stepper();
    }
}

static MotionLaserMode current_laser_mode(void)
{
    if (s_gc.spindle_speed == 0u) {
        return MOTION_LASER_OFF;
    }
    if (s_gc.laser_mode == 3u) {
        return MOTION_LASER_CONSTANT;
    }
    if (s_gc.laser_mode == 4u) {
        return MOTION_LASER_DYNAMIC;
    }
    return MOTION_LASER_OFF;
}

static void apply_spindle_power(void)
{
    if (s_gc.spindle_speed == 0u) {
        MotionPlanner_SetLaserPower(0u);
        return;
    }
    uint32_t power = ((uint32_t)s_gc.spindle_speed * APP_LASER_POWER_MAX_PERMILLE + 999u) / 1000u;
    if (power < APP_LASER_POWER_MIN_PERMILLE) {
        power = APP_LASER_POWER_MIN_PERMILLE;
    }
    MotionPlanner_SetLaserPower((uint16_t)power);
    (void)LaserControl_SetPowerPermille((uint16_t)power);
}

static bool arc_center_from_radius(int32_t sx,
                                   int32_t sy,
                                   int32_t ex,
                                   int32_t ey,
                                   int32_t radius_um,
                                   bool clockwise,
                                   int32_t *cx,
                                   int32_t *cy)
{
    float dx = (float)(ex - sx);
    float dy = (float)(ey - sy);
    float chord = hypotf(dx, dy);
    float radius = fabsf((float)radius_um);
    if (chord < 0.001f || radius < chord * 0.5f) {
        return false;
    }
    float mx = 0.5f * ((float)sx + (float)ex);
    float my = 0.5f * ((float)sy + (float)ey);
    float h = sqrtf(radius * radius - 0.25f * chord * chord);
    float nx = -dy / chord;
    float ny = dx / chord;
    float sign = clockwise ? -1.0f : 1.0f;
    if (radius_um < 0) {
        sign = -sign;
    }
    *cx = (int32_t)lroundf(mx + sign * h * nx);
    *cy = (int32_t)lroundf(my + sign * h * ny);
    return true;
}

static bool process_block(const char *line)
{
    int32_t g_code = -1;
    int32_t m_code = -1;
    int32_t x_um = s_gc.x_um;
    int32_t y_um = s_gc.y_um;
    int32_t i_um = 0;
    int32_t j_um = 0;
    int32_t r_um = 0;
    int32_t feed = s_gc.feed_mm_min;
    int32_t p_ms = 0;
    int32_t s_word = s_gc.spindle_speed;
    int32_t a_pulse = 0;
    int32_t b_pulse = 0;
    int32_t x_word_um = 0;
    int32_t y_word_um = 0;
    uint8_t seen_x = 0u;
    uint8_t seen_y = 0u;
    uint8_t seen_i = 0u;
    uint8_t seen_j = 0u;
    uint8_t seen_r = 0u;
    uint8_t seen_a = 0u;
    uint8_t seen_b = 0u;
    uint8_t coordinate_set = 0u;
    uint8_t jog = 0u;

    while (*line != '\0') {
        while (*line != '\0' && isspace((unsigned char)*line)) {
            line++;
        }
        if (*line == '\0' || *line == ';' || *line == '(') {
            break;
        }
        if (line[0] == '$' && toupper((unsigned char)line[1]) == 'J' && line[2] == '=') {
            jog = 1u;
            line += 3;
            continue;
        }
        char letter = (char)toupper((unsigned char)*line++);
        int32_t value = 0;
        if (letter == 'G') {
            if (!parse_int_word(&line, &value)) {
                send_error(2);
                return true;
            }
            if (value == 0 || value == 1 || value == 2 || value == 3) {
                g_code = value;
                if (!jog) {
                    s_gc.motion_mode = (uint8_t)value;
                }
            } else if (value == 4) {
                g_code = 4;
            } else if (value == 20) {
                if (!jog) {
                    s_gc.mm_units = 0u;
                }
            } else if (value == 21) {
                if (!jog) {
                    s_gc.mm_units = 1u;
                }
            } else if (value == 90) {
                if (!jog) {
                    s_gc.absolute = 1u;
                }
            } else if (value == 91) {
                if (!jog) {
                    s_gc.absolute = 0u;
                }
            } else if (value == 92) {
                coordinate_set = 1u;
            } else {
                send_error(20);
                return true;
            }
        } else if (letter == 'M') {
            if (!parse_int_word(&line, &m_code)) {
                send_error(2);
                return true;
            }
        } else if (letter == 'X' || letter == 'Y' || letter == 'I' || letter == 'J' || letter == 'R') {
            int32_t scaled;
            if (!parse_decimal_scaled(&line, s_gc.mm_units ? 1000 : 25400, &scaled)) {
                send_error(2);
                return true;
            }
            if (letter == 'X') {
                if (seen_x) { send_error(25); return true; }
                seen_x = 1u;
                x_word_um = scaled;
                x_um = (s_gc.absolute && !jog)
                           ? scaled + s_gc.offset_x_um
                           : s_gc.x_um + scaled;
            } else if (letter == 'Y') {
                if (seen_y) { send_error(25); return true; }
                seen_y = 1u;
                y_word_um = scaled;
                y_um = (s_gc.absolute && !jog)
                           ? scaled + s_gc.offset_y_um
                           : s_gc.y_um + scaled;
            } else if (letter == 'I') {
                seen_i = 1u;
                i_um = scaled;
            } else if (letter == 'J') {
                seen_j = 1u;
                j_um = scaled;
            } else {
                seen_r = 1u;
                r_um = scaled;
            }
        } else if (letter == 'F') {
            if (!parse_decimal_scaled(&line, 1, &feed) || feed <= 0) {
                send_error(4);
                return true;
            }
        } else if (letter == 'P') {
            if (!parse_decimal_scaled(&line, 1000, &p_ms) || p_ms < 0) {
                send_error(4);
                return true;
            }
        } else if (letter == 'S') {
            if (!parse_decimal_scaled(&line, 1, &s_word) || s_word < 0 || s_word > 1000) {
                send_error(4);
                return true;
            }
        } else if (letter == 'A' || letter == 'B') {
            if (!parse_decimal_scaled(&line, 1, &value)) {
                send_error(2);
                return true;
            }
            if (letter == 'A') {
                seen_a = 1u;
                a_pulse = value;
            } else {
                seen_b = 1u;
                b_pulse = value;
            }
        } else if (letter == 'N') {
            if (!parse_int_word(&line, &value)) {
                send_error(2);
                return true;
            }
        } else {
            send_error(20);
            return true;
        }
    }

    if (!jog) {
        s_gc.feed_mm_min = feed;
        s_gc.spindle_speed = (uint16_t)s_word;
        apply_spindle_power();
    }

    if (m_code >= 0) {
        if (m_code == 17) {
            Stepper_EnableAll(true);
        } else if (m_code == 18) {
            Stepper_EnableAll(false);
        } else if (m_code == 3 || m_code == 4) {
            if (!MotionPlanner_PlanDwell(1u, MOTION_LASER_OFF)) {
                send_error(8);
                return true;
            }
            s_gc.laser_mode = (uint8_t)m_code;
        } else if (m_code == 5) {
            if (!MotionPlanner_PlanDwell(1u, MOTION_LASER_OFF)) {
                send_error(8);
                return true;
            }
            s_gc.laser_mode = 0u;
        } else if (m_code == 0 || m_code == 2 || m_code == 30) {
            if (!MotionPlanner_PlanDwell(1u, MOTION_LASER_OFF)) {
                send_error(8);
                return true;
            }
            if (m_code == 2 || m_code == 30) {
                s_gc.laser_mode = 0u;
            }
        } else if (m_code == 112) {
            SerialDma_FlushRxLines();
            GcodeStream_Clear();
            HomeController_Stop();
            Stepper_EStopAll();
        } else {
            send_error(20);
            return true;
        }
    }

    if (coordinate_set) {
        if (MotionPlanner_IsBusy()) {
            send_error(8);
            return true;
        }
        MotionPlanner_Clear();
        if (seen_a) {
            Stepper_SetPosition(STEPPER_AXIS_1, a_pulse);
        }
        if (seen_b) {
            Stepper_SetPosition(STEPPER_AXIS_2, b_pulse);
        }
        if (seen_a || seen_b) {
            sync_position_from_stepper();
        }
        if (seen_x) {
            s_gc.offset_x_um = s_gc.x_um - x_word_um;
        }
        if (seen_y) {
            s_gc.offset_y_um = s_gc.y_um - y_word_um;
        }
        send_ok();
        return true;
    }
    if (seen_a || seen_b) {
        send_error(20);
        return true;
    }
    if (g_code == 4) {
        if (!MotionPlanner_PlanDwell((uint32_t)p_ms, current_laser_mode())) {
            send_error(8);
            return true;
        }
        send_ok();
        return true;
    }
    if (!seen_x && !seen_y) {
        send_ok();
        return true;
    }
    if (jog && MotionPlanner_IsBusy()) {
        send_error(8);
        return true;
    }
    if (MotionPlanner_Free() == 0u) {
        send_error(8);
        return true;
    }

    bool accepted;
    uint8_t motion = jog ? 1u : (g_code >= 0 ? (uint8_t)g_code : s_gc.motion_mode);
    if (motion == 2u || motion == 3u) {
        int32_t cx;
        int32_t cy;
        if (seen_i || seen_j) {
            cx = s_gc.x_um + i_um;
            cy = s_gc.y_um + j_um;
        } else if (seen_r && arc_center_from_radius(s_gc.x_um, s_gc.y_um, x_um, y_um, r_um, motion == 2u, &cx, &cy)) {
            /* Center computed from R. */
        } else {
            send_error(33);
            return true;
        }
        accepted = MotionPlanner_PlanArc(s_gc.x_um, s_gc.y_um, x_um, y_um, cx, cy,
                                         motion == 2u, feed, jog != 0u, current_laser_mode());
    } else {
        accepted = MotionPlanner_PlanLine(s_gc.x_um, s_gc.y_um, x_um, y_um, feed,
                                          motion == 0u, jog != 0u, current_laser_mode());
    }
    if (!accepted) {
        send_error(15);
        return true;
    }
    s_gc.x_um = x_um;
    s_gc.y_um = y_um;
    if (jog) {
        /*
         * Defer the jog ACK until the move actually finishes. The host serializes
         * jogs on this "ok": acknowledging at planner-accept time lets a reversing
         * jog be issued while the previous one is still draining the segment/stepper
         * FIFO, where MotionPlanner_IsBusy() rejects it (error:8) and it is silently
         * dropped, leaving the arm one full jog step short. GcodeStream_Loop emits
         * the ok once motion is complete.
         */
        s_jog_pending_ack = 1u;
        return true;
    }
    send_ok();
    return true;
}

void GcodeStream_Init(void)
{
    memset(&s_gc, 0, sizeof(s_gc));
    s_gc.absolute = 1u;
    s_gc.mm_units = 1u;
    s_gc.motion_mode = 0u;
    s_gc.feed_mm_min = APP_GCODE_DEFAULT_FEED_MM_MIN;
    s_status_due = 0u;
    s_status_divider = 0u;
    s_hold = 0u;
    s_home_pending_ack = 0u;
    s_jog_pending_ack = 0u;
    s_motion_settle_pending = 0u;
    sync_position_from_stepper();
}

void GcodeStream_Clear(void)
{
    MotionPlanner_Clear();
    s_home_pending_ack = 0u;
    s_jog_pending_ack = 0u;
    s_motion_settle_pending = 0u;
    sync_position_from_stepper();
}

uint8_t GcodeStream_PlannerFree(void) { return MotionPlanner_Free(); }
uint8_t GcodeStream_PlannerCount(void) { return MotionPlanner_Count(); }
bool GcodeStream_HomeAckPending(void) { return s_home_pending_ack != 0u; }

void GcodeStream_RequestStatus(void)
{
    s_status_due = 1u;
}

static void send_status(void)
{
    StepperState state;
    HomeSensorState home;
    MotionPlannerSnapshot planner;
    LaserControlState laser;
    ScaraPose pose = {s_gc.x_um, s_gc.y_um};
    Stepper_GetStateSnapshot(&state);
    HomeSensor_GetState(&home);
    MotionPlanner_GetSnapshot(&planner);
    LaserControl_GetState(&laser);
    (void)ScaraKinematics_PulseToPose(state.axis[0].position_pulse, state.axis[1].position_pulse, &pose);
    const char *mode = s_hold ? "Hold" : (MotionPlanner_IsBusy() ? "Run" : "Idle");
    const char *home_state = HomeController_StateName(HomeController_GetState());
    uint32_t stepper_error = state.axis[0].error | state.axis[1].error;
    SerialDma_SendFormat("<%s|MPos:%ld.%03ld,%ld.%03ld|JPos:%ld,%ld|FS:%ld,%u|Bf:%u,%lu|Q:%u|E:%lu|Seg:%u,%u,%u,%lu|Pf:%lu|Rl:%lu|Pg:%lu|H:%u,%u|HS:%s|A1:%u,%u,%ld,%ld|A2:%u,%u,%ld,%ld|Lz:%u,%u,%u,%u>\n",
                         mode,
                         (long)(pose.x_um / 1000), (long)i32_abs(pose.x_um % 1000),
                         (long)(pose.y_um / 1000), (long)i32_abs(pose.y_um % 1000),
                         (long)state.axis[0].position_pulse,
                         (long)state.axis[1].position_pulse,
                         (long)s_gc.feed_mm_min,
                         (unsigned int)s_gc.spindle_speed,
                         (unsigned int)planner.planner_free,
                         (unsigned long)SerialDma_RxFreeBytes(),
                         (unsigned int)planner.planner_count,
                         (unsigned long)stepper_error,
                         (unsigned int)planner.segment_count,
                         (unsigned int)planner.segment_free,
                         (unsigned int)planner.segment_low_water,
                         (unsigned long)planner.segment_underrun_count,
                         (unsigned long)planner.preparation_fault_count,
                         (unsigned long)planner.rate_limited_segment_count,
                         (unsigned long)planner.max_refill_gap_ms,
                         home.home1_active ? 1u : 0u,
                         home.home2_active ? 1u : 0u,
                         home_state,
                         state.axis[0].enabled ? 1u : 0u,
                         state.axis[0].running ? 1u : 0u,
                         (long)state.axis[0].current_pps,
                         (long)state.axis[0].target_pps,
                         state.axis[1].enabled ? 1u : 0u,
                         state.axis[1].running ? 1u : 0u,
                         (long)state.axis[1].current_pps,
                         (long)state.axis[1].target_pps,
                         laser.armed ? 1u : 0u,
                         laser.relay_ready ? 1u : 0u,
                         laser.marking ? 1u : 0u,
                         (unsigned int)laser.power_permille);
}

void GcodeStream_Loop(void)
{
    if (s_home_pending_ack && !SerialDma_IsTxBusy()) {
        HomeControllerState home_state = HomeController_GetState();
        if (home_state == HOME_CTRL_DONE) {
            sync_position_from_stepper();
            s_home_pending_ack = 0u;
            send_ok();
            return;
        }
        if (home_state == HOME_CTRL_ERROR) {
            s_home_pending_ack = 0u;
            send_error(5);
            return;
        }
    }
    if (!s_home_pending_ack) {
        /* Re-anchor the parser to the executed pulses on the busy->idle edge only,
         * so mid-stream the parser stays at the last commanded endpoint. */
        if (MotionPlanner_IsBusy()) {
            s_motion_settle_pending = 1u;
        } else if (s_motion_settle_pending) {
            s_motion_settle_pending = 0u;
            resync_parser_to_stepper_if_diverged();
        }
    }
    if (s_jog_pending_ack && !SerialDma_IsTxBusy() && !MotionPlanner_IsBusy()) {
        /* Jog motion has fully drained (planner empty and stepper idle). */
        s_jog_pending_ack = 0u;
        send_ok();
        return;
    }
    if (s_status_due && !SerialDma_IsTxBusy()) {
        s_status_due = 0u;
        send_status();
    }
}

void GcodeStream_Tick1kHz(void)
{
    s_status_divider++;
    if (s_status_divider >= APP_GCODE_STATUS_PERIOD_MS) {
        s_status_divider = 0u;
        s_status_due = 1u;
    }
}

static bool process_setting(const char *line)
{
    long value = 0;
    long value2 = 0;
    if (sscanf(line, "$100=%ld $101=%ld", &value, &value2) == 2) {
        if (value <= 0 || value2 <= 0 || MotionPlanner_IsBusy()) {
            send_error((value <= 0 || value2 <= 0) ? 4u : 8u);
            return true;
        }
        AppParams *params = AppParams_Mutable();
        params->pulses_per_rev[0] = (int32_t)value;
        params->pulses_per_rev[1] = (int32_t)value2;
        send_ok();
        return true;
    }
    if (sscanf(line, "$100=%ld", &value) == 1 || sscanf(line, "$101=%ld", &value) == 1) {
        if (value <= 0 || MotionPlanner_IsBusy()) {
            send_error(value <= 0 ? 4u : 8u);
            return true;
        }
        AppParams *params = AppParams_Mutable();
        if (line[3] == '0') {
            params->pulses_per_rev[0] = (int32_t)value;
        } else {
            params->pulses_per_rev[1] = (int32_t)value;
        }
        send_ok();
        return true;
    }
    if (sscanf(line, "$11=%ld", &value) == 1) {
        MotionPlanner_SetJunctionDeviation((float)value / 1000.0f);
        send_ok();
        return true;
    }
    if (sscanf(line, "$12=%ld", &value) == 1) {
        MotionPlanner_SetArcTolerance((float)value / 1000.0f);
        send_ok();
        return true;
    }
    if (sscanf(line, "$110=%ld", &value) == 1 || sscanf(line, "$111=%ld", &value) == 1) {
        if (value <= 0 || MotionPlanner_IsBusy()) {
            send_error(value <= 0 ? 4u : 8u);
            return true;
        }
        MotionPlanner_SetMaxFeed((float)value);
        send_ok();
        return true;
    }
    if (sscanf(line, "$120=%ld", &value) == 1 || sscanf(line, "$121=%ld", &value) == 1) {
        if (value <= 0 || MotionPlanner_IsBusy()) {
            send_error(value <= 0 ? 4u : 8u);
            return true;
        }
        MotionPlanner_SetAcceleration((float)value);
        send_ok();
        return true;
    }
    if (strcmp(line, "$$") == 0) {
        const AppParams *params = AppParams_Get();
        SerialDma_SendFormat("$11=%ld\n$12=%ld\n$100=%ld\n$101=%ld\n$110=%ld\n$111=%ld\n$120=%ld\n$121=%ld\nok\n",
                             (long)lroundf(MotionPlanner_GetJunctionDeviation() * 1000.0f),
                             (long)lroundf(MotionPlanner_GetArcTolerance() * 1000.0f),
                             (long)params->pulses_per_rev[0],
                             (long)params->pulses_per_rev[1],
                             (long)lroundf(MotionPlanner_GetMaxFeed()),
                             (long)lroundf(MotionPlanner_GetMaxFeed()),
                             (long)lroundf(MotionPlanner_GetAcceleration()),
                             (long)lroundf(MotionPlanner_GetAcceleration()));
        return true;
    }
    return false;
}

bool GcodeStream_TryProcessLine(const char *line)
{
    if (line == NULL || !starts_gcode(line)) {
        return false;
    }
    while (*line != '\0' && isspace((unsigned char)*line)) {
        line++;
    }
    if (line[0] == '?') {
        GcodeStream_RequestStatus();
        return true;
    }
    if (line[0] == '!') {
        s_hold = 1u;
        MotionPlanner_SetHold(true);
        return true;
    }
    if (line[0] == '~') {
        s_hold = 0u;
        MotionPlanner_SetHold(false);
        return true;
    }
    if ((unsigned char)line[0] == 0x18u) {
        SerialDma_FlushRxLines();
        GcodeStream_Clear();
        HomeController_Stop();
        HomeController_ClearError();
        Stepper_StopAll();
        Stepper_ClearError();
        /* 软复位后把解析器位置重锚到电机真实停点（与 0x85/坐标设定路径一致）；
         * 否则中断停车后 s_gc 仍停在旧目标，下一条运动准备阶段会立即 preparation_fault。 */
        resync_parser_to_stepper_if_diverged();
        s_hold = 0u;
        return true;
    }
    if ((unsigned char)line[0] == 0x85u) {
        MotionPlanner_Stop();
        sync_position_from_stepper();
        s_hold = 0u;
        return true;
    }
    if (line[0] == '$') {
        if (toupper((unsigned char)line[1]) == 'J' && line[2] == '=') {
            return process_block(line);
        }
        if (toupper((unsigned char)line[1]) == 'X') {
            Stepper_ClearError();
            HomeController_ClearError();
            send_ok();
            return true;
        }
        if (toupper((unsigned char)line[1]) == 'H') {
            if (MotionPlanner_IsBusy()) {
                send_error(8);
            } else if (HomeController_Start(is_home_sim_command(line))) {
                MotionPlanner_Clear();
                s_home_pending_ack = 1u;
            } else {
                send_error(5);
            }
            return true;
        }
        if (toupper((unsigned char)line[1]) == 'G') {
            SerialDma_SendFormat("[GC:G%u G%u G%u M%u F%ld S%u]\nok\n",
                                 (unsigned int)s_gc.motion_mode,
                                 s_gc.absolute ? 90u : 91u,
                                 s_gc.mm_units ? 21u : 20u,
                                 (unsigned int)s_gc.laser_mode,
                                 (long)s_gc.feed_mm_min,
                                 (unsigned int)s_gc.spindle_speed);
            return true;
        }
        if (process_setting(line)) {
            return true;
        }
        send_error(3);
        return true;
    }
    return process_block(line);
}
