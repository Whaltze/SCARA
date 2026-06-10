#include "gcode_stream.h"

/* G-code 流接收层：解析上位机已经规划好的 G0/G1 X/Y/F，并返回 ok seq/cs/line 回显。 */

#include "app_config.h"
#include "app_main.h"
#include "app_params.h"
#include "binary_traj.h"
#include "home_controller.h"
#include "home_sensor.h"
#include "motion_planner.h"
#include "scara_kinematics.h"
#include "serial_dma.h"
#include "stepper_driver.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    /* 规划块只保存执行所需信息；速度和限位规划由上位机提前完成。 */
    int32_t x_um;
    int32_t y_um;
    int32_t dx_um;
    int32_t dy_um;
    int32_t p1;
    int32_t p2;
    int16_t v1_pps;
    int16_t v2_pps;
    int16_t exit1_pps;
    int16_t exit2_pps;
    uint16_t duration_ticks;
    uint8_t rapid;
    uint8_t timed;
} GcodeBlock;

typedef struct {
    uint8_t absolute;
    uint8_t mm_units;
    uint8_t motion_mode;
    int32_t feed_mm_min;
    int32_t x_um;
    int32_t y_um;
    int32_t p1;
    int32_t p2;
} GcodeState;

static GcodeBlock s_blocks[APP_GCODE_PLANNER_BLOCKS];
static uint8_t s_head;
static uint8_t s_tail;
static uint8_t s_count;
static GcodeState s_gc;
static char s_pending[APP_SERIAL_LINE_SIZE];
static uint8_t s_pending_valid;
static volatile uint8_t s_status_due;
static uint16_t s_status_divider;
static uint8_t s_hold;
static uint32_t s_ack_seq;
static uint16_t s_start_wait_ms;

static uint8_t next_index(uint8_t index)
{
    index++;
    if (index >= APP_GCODE_PLANNER_BLOCKS) {
        index = 0;
    }
    return index;
}

static uint8_t prev_index(uint8_t index)
{
    if (index == 0u) {
        return APP_GCODE_PLANNER_BLOCKS - 1u;
    }
    return (uint8_t)(index - 1u);
}

static int64_t i64_abs(int64_t v)
{
    return v < 0 ? -v : v;
}

static int32_t i32_abs(int32_t v)
{
    return v < 0 ? -v : v;
}

static int32_t i32_isqrt_i64(int64_t value)
{
    int64_t bit = 1LL << 62;
    int64_t result = 0;

    if (value <= 0) {
        return 0;
    }
    while (bit > value) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    if (result > 2147483647LL) {
        return 2147483647;
    }
    return (int32_t)result;
}

static int16_t pps_to_i16(int32_t pps)
{
    if (pps > 30000) {
        return 30000;
    }
    if (pps < -30000) {
        return -30000;
    }
    return (int16_t)pps;
}

static bool starts_gcode(const char *line)
{
    while (*line != '\0' && isspace((unsigned char)*line)) {
        line++;
    }
    char ch = (char)toupper((unsigned char)*line);
    return ch == 'G' || ch == 'M' || ch == '$' || ch == '?' || ch == '!' || ch == '~' || (unsigned char)ch == 0x18u;
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
    int sign = 1;
    int64_t whole = 0;
    int64_t frac = 0;
    int32_t frac_scale = scale;
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
            if (frac_scale > 1) {
                frac_scale /= 10;
                frac += (int64_t)(**cursor - '0') * frac_scale;
            }
            (*cursor)++;
        }
    }

    if (!digit) {
        return false;
    }

    int64_t value = whole * scale + frac;
    if (sign < 0) {
        value = -value;
    }
    *out = (int32_t)value;
    return true;
}

static void send_error(uint8_t code)
{
    SerialDma_SendFormat("error:%u\n", (unsigned int)code);
}

static uint8_t line_checksum(const char *line)
{
    uint8_t sum = 0;
    while (*line != '\0') {
        sum = (uint8_t)(sum + (uint8_t)*line);
        line++;
    }
    return sum;
}

static void send_ok_for_line(const char *line)
{
    /* 每条已接收指令必须完整回显，方便上位机确认高频通信没有错包/串包。 */
    s_ack_seq++;
    SerialDma_SendFormat("ok seq=%lu cs=%02X line=%s\n",
                         (unsigned long)s_ack_seq,
                         (unsigned int)line_checksum(line),
                         line);
}

static bool buffer_full(void)
{
    return s_count >= APP_GCODE_PLANNER_BLOCKS;
}

static void enqueue_block(const GcodeBlock *block)
{
    /* 下位机不做角点降速，只根据相邻段的上位机给定速度做薄执行拼接。 */
    GcodeBlock planned = *block;
    planned.exit1_pps = 0;
    planned.exit2_pps = 0;
    if (s_count > 0u && !planned.rapid && !planned.timed) {
        GcodeBlock *prev = &s_blocks[prev_index(s_head)];
        if (!prev->rapid && !prev->timed) {
            int32_t exit1 = i32_abs(prev->v1_pps) < i32_abs(planned.v1_pps) ? i32_abs(prev->v1_pps) : i32_abs(planned.v1_pps);
            int32_t exit2 = i32_abs(prev->v2_pps) < i32_abs(planned.v2_pps) ? i32_abs(prev->v2_pps) : i32_abs(planned.v2_pps);
            if (exit1 > 0 && exit1 < APP_MIN_EFFECTIVE_PPS) {
                exit1 = APP_MIN_EFFECTIVE_PPS;
            }
            if (exit2 > 0 && exit2 < APP_MIN_EFFECTIVE_PPS) {
                exit2 = APP_MIN_EFFECTIVE_PPS;
            }
            prev->exit1_pps = pps_to_i16(exit1);
            prev->exit2_pps = pps_to_i16(exit2);
        }
    }
    if (!planned.timed && planned.v1_pps > 0 && planned.v1_pps < APP_MIN_EFFECTIVE_PPS) {
        planned.v1_pps = APP_MIN_EFFECTIVE_PPS;
    }
    if (!planned.timed && planned.v2_pps > 0 && planned.v2_pps < APP_MIN_EFFECTIVE_PPS) {
        planned.v2_pps = APP_MIN_EFFECTIVE_PPS;
    }

    s_blocks[s_head] = planned;
    s_head = next_index(s_head);
    s_count++;
    if (s_count == 1u) {
        s_start_wait_ms = 0;
    }
    s_gc.x_um = block->x_um;
    s_gc.y_um = block->y_um;
    s_gc.p1 = block->p1;
    s_gc.p2 = block->p2;
}

static bool current_xy_from_stepper(int32_t *x_um, int32_t *y_um)
{
    StepperState state;
    ScaraJoint joint;
    ScaraPose pose;
    Stepper_GetStateSnapshot(&state);
    if (!ScaraKinematics_PulseToJoint(state.axis[0].position_pulse, state.axis[1].position_pulse, &joint)) {
        return false;
    }
    if (!ScaraKinematics_Forward(&joint, &pose)) {
        return false;
    }
    *x_um = pose.x_um;
    *y_um = pose.y_um;
    return true;
}

static void sync_position_from_stepper(void)
{
    StepperState state;
    Stepper_GetStateSnapshot(&state);
    s_gc.p1 = (int32_t)state.axis[0].position_pulse;
    s_gc.p2 = (int32_t)state.axis[1].position_pulse;
    (void)current_xy_from_stepper(&s_gc.x_um, &s_gc.y_um);
}

static bool build_motion_block(int32_t target_x_um, int32_t target_y_um, uint8_t rapid, GcodeBlock *out)
{
    /* 逆解和脉冲换算在主循环中完成，TIM 中断不做浮点/三角计算。 */
    ScaraJoint joint;
    int64_t p1 = 0;
    int64_t p2 = 0;
    if (!ScaraKinematics_InverseUm(target_x_um, target_y_um, &joint) ||
        !ScaraKinematics_JointToPulse(joint.theta1_mrad, joint.theta2_mrad, &p1, &p2) ||
        !Stepper_TargetsAllowed(p1, p2)) {
        return false;
    }

    int64_t dp1 = p1 - (int64_t)s_gc.p1;
    int64_t dp2 = p2 - (int64_t)s_gc.p2;
    int32_t dx = target_x_um - s_gc.x_um;
    int32_t dy = target_y_um - s_gc.y_um;
    int32_t dist_um = i32_isqrt_i64((int64_t)dx * (int64_t)dx + (int64_t)dy * (int64_t)dy);
    if (dist_um < 1) {
        dist_um = 1;
    }

    int32_t v1 = APP_GCODE_RAPID_PPS;
    int32_t v2 = APP_GCODE_RAPID_PPS;
    if (!rapid) {
        int32_t feed = s_gc.feed_mm_min > 0 ? s_gc.feed_mm_min : APP_GCODE_DEFAULT_FEED_MM_MIN;
        int64_t n1 = i64_abs(dp1) * (int64_t)feed * 50;
        int64_t n2 = i64_abs(dp2) * (int64_t)feed * 50;
        v1 = (int32_t)(n1 / ((int64_t)dist_um * 3));
        v2 = (int32_t)(n2 / ((int64_t)dist_um * 3));
        if (v1 < APP_MIN_EFFECTIVE_PPS && dp1 != 0) {
            v1 = APP_MIN_EFFECTIVE_PPS;
        }
        if (v2 < APP_MIN_EFFECTIVE_PPS && dp2 != 0) {
            v2 = APP_MIN_EFFECTIVE_PPS;
        }
        if (v1 > APP_MAX_PPS_DEFAULT) {
            v1 = APP_MAX_PPS_DEFAULT;
        }
        if (v2 > APP_MAX_PPS_DEFAULT) {
            v2 = APP_MAX_PPS_DEFAULT;
        }
    }

    out->x_um = target_x_um;
    out->y_um = target_y_um;
    out->dx_um = dx;
    out->dy_um = dy;
    out->p1 = (int32_t)p1;
    out->p2 = (int32_t)p2;
    out->v1_pps = pps_to_i16(v1);
    out->v2_pps = pps_to_i16(v2);
    out->exit1_pps = 0;
    out->exit2_pps = 0;
    out->duration_ticks = 0;
    out->rapid = rapid;
    out->timed = 0;
    return true;
}

static bool build_timed_block(int32_t p1, int32_t p2, uint16_t duration_ticks, GcodeBlock *out)
{
    int32_t d1 = p1 - s_gc.p1;
    int32_t d2 = p2 - s_gc.p2;
    uint32_t events = (uint32_t)(i32_abs(d1) > i32_abs(d2) ? i32_abs(d1) : i32_abs(d2));

    if (duration_ticks == 0u || events > (uint32_t)duration_ticks) {
        return false;
    }
    if (!Stepper_TargetsAllowed(p1, p2)) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->x_um = s_gc.x_um;
    out->y_um = s_gc.y_um;
    out->p1 = p1;
    out->p2 = p2;
    out->duration_ticks = duration_ticks;
    out->timed = 1;
    return true;
}

static bool process_block(const char *line, uint8_t send_ok_now)
{
    const char *original_line = line;
    int32_t m_code = -1;
    int32_t x_um = s_gc.x_um;
    int32_t y_um = s_gc.y_um;
    int32_t feed = s_gc.feed_mm_min;
    int32_t a_pulse = s_gc.p1;
    int32_t b_pulse = s_gc.p2;
    int32_t t_ticks = 0;
    uint8_t seen_x = 0;
    uint8_t seen_y = 0;
    uint8_t seen_a = 0;
    uint8_t seen_b = 0;
    uint8_t seen_t = 0;
    uint8_t has_motion_word = 0;
    uint8_t dwell = 0;
    uint8_t coordinate_set = 0;

    while (*line != '\0') {
        while (*line != '\0' && (isspace((unsigned char)*line) || *line == ';')) {
            if (*line == ';') {
                break;
            }
            line++;
        }
        if (*line == '\0' || *line == ';' || *line == '(') {
            break;
        }

        char letter = (char)toupper((unsigned char)*line++);
        if (letter == 'G') {
            int32_t value = 0;
            if (!parse_int_word(&line, &value)) {
                send_error(2);
                return true;
            }
            if (value == 0 || value == 1) {
                s_gc.motion_mode = (uint8_t)value;
                has_motion_word = 1;
            } else if (value == 92) {
                coordinate_set = 1;
            } else if (value == 90) {
                s_gc.absolute = 1;
            } else if (value == 91) {
                s_gc.absolute = 0;
            } else if (value == 20) {
                s_gc.mm_units = 0;
            } else if (value == 21) {
                s_gc.mm_units = 1;
            } else if (value == 4) {
                dwell = 1;
            } else {
                send_error(20);
                return true;
            }
        } else if (letter == 'M') {
            if (!parse_int_word(&line, &m_code)) {
                send_error(2);
                return true;
            }
            if (!(m_code == 0 || m_code == 2 || m_code == 17 || m_code == 18 || m_code == 30 || m_code == 112)) {
                send_error(20);
                return true;
            }
        } else if (letter == 'X' || letter == 'Y') {
            int32_t value_um = 0;
            if (!parse_decimal_scaled(&line, s_gc.mm_units ? 1000 : 25400, &value_um)) {
                send_error(2);
                return true;
            }
            if (letter == 'X') {
                if (seen_x) {
                    send_error(25);
                    return true;
                }
                seen_x = 1;
                x_um = s_gc.absolute ? value_um : s_gc.x_um + value_um;
            } else {
                if (seen_y) {
                    send_error(25);
                    return true;
                }
                seen_y = 1;
                y_um = s_gc.absolute ? value_um : s_gc.y_um + value_um;
            }
        } else if (letter == 'F') {
            int32_t value = 0;
            if (!parse_decimal_scaled(&line, 1, &value) || value <= 0) {
                send_error(4);
                return true;
            }
            feed = value;
        } else if (letter == 'A' || letter == 'B') {
            int32_t value = 0;
            if (!parse_decimal_scaled(&line, 1, &value)) {
                send_error(2);
                return true;
            }
            if (letter == 'A') {
                if (seen_a) {
                    send_error(25);
                    return true;
                }
                seen_a = 1;
                a_pulse = s_gc.absolute ? value : s_gc.p1 + value;
            } else {
                if (seen_b) {
                    send_error(25);
                    return true;
                }
                seen_b = 1;
                b_pulse = s_gc.absolute ? value : s_gc.p2 + value;
            }
        } else if (letter == 'T') {
            int32_t value = 0;
            if (!parse_decimal_scaled(&line, 1, &value) || value <= 0 || value > 65535) {
                send_error(4);
                return true;
            }
            seen_t = 1;
            t_ticks = value;
        } else if (letter == 'P') {
            int32_t unused = 0;
            if (!parse_decimal_scaled(&line, 1000, &unused)) {
                send_error(2);
                return true;
            }
        } else if (letter == 'N') {
            int32_t unused = 0;
            if (!parse_int_word(&line, &unused)) {
                send_error(2);
                return true;
            }
        } else {
            send_error(20);
            return true;
        }
    }

    s_gc.feed_mm_min = feed;

    if (m_code == 17) {
        Stepper_EnableAll(true);
        if (send_ok_now) {
            send_ok_for_line(original_line);
        }
        return true;
    }
    if (m_code == 18) {
        Stepper_EnableAll(false);
        if (send_ok_now) {
            send_ok_for_line(original_line);
        }
        return true;
    }
    if (m_code == 112) {
        GcodeStream_Clear();
        HomeController_Stop();
        Stepper_EStopAll();
        if (send_ok_now) {
            send_ok_for_line(original_line);
        }
        return true;
    }

    if (coordinate_set) {
        if (Stepper_IsBusy() || s_count > 0u) {
            send_error(8);
            return true;
        }
        if (seen_a) {
            Stepper_SetPosition(STEPPER_AXIS_1, a_pulse);
            s_gc.p1 = a_pulse;
        }
        if (seen_b) {
            Stepper_SetPosition(STEPPER_AXIS_2, b_pulse);
            s_gc.p2 = b_pulse;
        }
        sync_position_from_stepper();
        if (send_ok_now) {
            send_ok_for_line(original_line);
        }
        return true;
    }

    if (seen_t || seen_a || seen_b) {
        if (!(seen_a && seen_b && seen_t)) {
            send_error(2);
            return true;
        }
        GcodeBlock block;
        if (!build_timed_block(a_pulse, b_pulse, (uint16_t)t_ticks, &block)) {
            send_error(15);
            return true;
        }
        enqueue_block(&block);
        if (send_ok_now) {
            send_ok_for_line(original_line);
        }
        return true;
    }

    if (dwell || m_code == 0 || m_code == 2 || m_code == 30 || (!seen_x && !seen_y && !has_motion_word)) {
        if (send_ok_now) {
            send_ok_for_line(original_line);
        }
        return true;
    }

    GcodeBlock block;
    if (!build_motion_block(x_um, y_um, s_gc.motion_mode == 0u, &block)) {
        send_error(15);
        return true;
    }
    enqueue_block(&block);
    if (send_ok_now) {
        send_ok_for_line(original_line);
    }
    return true;
}

void GcodeStream_Init(void)
{
    s_head = 0;
    s_tail = 0;
    s_count = 0;
    s_pending_valid = 0;
    s_status_due = 0;
    s_status_divider = 0;
    s_hold = 0;
    s_ack_seq = 0;
    s_start_wait_ms = 0;
    memset(&s_gc, 0, sizeof(s_gc));
    s_gc.absolute = 1;
    s_gc.mm_units = 1;
    s_gc.motion_mode = 0;
    s_gc.feed_mm_min = APP_GCODE_DEFAULT_FEED_MM_MIN;
    sync_position_from_stepper();
}

void GcodeStream_Clear(void)
{
    s_head = 0;
    s_tail = 0;
    s_count = 0;
    s_pending_valid = 0;
    s_start_wait_ms = 0;
}

uint8_t GcodeStream_PlannerFree(void)
{
    return (uint8_t)(APP_GCODE_PLANNER_BLOCKS - s_count);
}

uint8_t GcodeStream_PlannerCount(void)
{
    return s_count;
}

void GcodeStream_RequestStatus(void)
{
    s_status_due = 1;
}

static void send_status(void)
{
    StepperState state;
    HomeSensorState home;
    BinaryTrajSnapshot traj;
    int32_t x = s_gc.x_um;
    int32_t y = s_gc.y_um;
    Stepper_GetStateSnapshot(&state);
    HomeSensor_GetState(&home);
    BinaryTraj_GetSnapshot(&traj);
    (void)current_xy_from_stepper(&x, &y);
    const char *mode = Stepper_IsBusy() || s_count > 0 ? "Run" : "Idle";
    uint32_t err = state.axis[0].error | state.axis[1].error;
    SerialDma_SendFormat("<%s|M:%ld.%03ld,%ld.%03ld|P:%ld,%ld|Bf:%u,%u|Q:%u|Sq:%u,%u|JT:%s,%lu,%lu,%u,%u|JU:%lu,%lu,%u,%lu|E:%lu|H:%u,%u|HS:%s|Hz:%lu|IC:%lu|A1:%u,%u,%ld,%ld|A2:%u,%u,%ld,%ld>\n",
                         mode,
                         (long)(x / 1000), (long)i32_abs(x % 1000),
                         (long)(y / 1000), (long)i32_abs(y % 1000),
                         (long)state.axis[0].position_pulse,
                         (long)state.axis[1].position_pulse,
                         (unsigned int)GcodeStream_PlannerFree(),
                         (unsigned int)SerialDma_RxFreeBytes(),
                         (unsigned int)GcodeStream_PlannerCount(),
                         (unsigned int)Stepper_TimedSegmentCount(),
                         (unsigned int)Stepper_TimedSegmentFree(),
                         BinaryTraj_StateName(traj.state),
                         (unsigned long)traj.accepted_count,
                         (unsigned long)traj.executed_count,
                         (unsigned int)traj.buffer_count,
                         (unsigned int)traj.buffer_free,
                         (unsigned long)traj.stream_underrun_ticks,
                         (unsigned long)traj.max_dispatch_gap_ticks,
                         (unsigned int)traj.min_buffer_count,
                         (unsigned long)traj.stream_underrun_count,
                         (unsigned long)err,
                         home.home1_active ? 1u : 0u,
                         home.home2_active ? 1u : 0u,
                         HomeController_StateName(HomeController_GetState()),
                         (unsigned long)APP_CONTROL_HZ,
                         (unsigned long)App_MaxTickCycles(),
                         state.axis[0].enabled ? 1u : 0u,
                         state.axis[0].running ? 1u : 0u,
                         (long)state.axis[0].current_pps,
                         (long)state.axis[0].target_pps,
                         state.axis[1].enabled ? 1u : 0u,
                         state.axis[1].running ? 1u : 0u,
                         (long)state.axis[1].current_pps,
                         (long)state.axis[1].target_pps);
}

void GcodeStream_Loop(void)
{
    /* 主循环消费待入队行和规划块；不会在中断里解析 G-code。 */
    if (s_pending_valid && !buffer_full()) {
        char pending[APP_SERIAL_LINE_SIZE];
        strncpy(pending, s_pending, sizeof(pending) - 1u);
        pending[sizeof(pending) - 1u] = '\0';
        s_pending_valid = 0;
        (void)process_block(pending, 1);
    }

    if (!s_hold && s_count > 0) {
        GcodeBlock *block = &s_blocks[s_tail];
        bool can_dispatch = block->timed ? Stepper_CanQueueTimedSegment() : Stepper_CanAcceptMove();
        uint8_t start_ready = block->timed || block->rapid || s_count >= 2u || s_start_wait_ms >= APP_GCODE_BLEND_START_DELAY_MS;
        bool started = false;
        if (can_dispatch) {
            if (start_ready && block->timed) {
                started = Stepper_MoveAbsTicks(block->p1, block->p2, block->duration_ticks);
            } else if (start_ready) {
                started = MotionPlanner_MoveAbsBlend(block->p1,
                                                     block->p2,
                                                     block->v1_pps,
                                                     block->v2_pps,
                                                     block->exit1_pps,
                                                     block->exit2_pps);
            }
            if (started) {
                s_tail = next_index(s_tail);
                s_count--;
                s_start_wait_ms = 0;
            } else {
                if (start_ready) {
                    send_error(15);
                    s_tail = next_index(s_tail);
                    s_count--;
                    s_start_wait_ms = 0;
                }
            }
        }
    }

    if (s_status_due && !SerialDma_IsTxBusy()) {
        s_status_due = 0;
        send_status();
    }
}

void GcodeStream_Tick1kHz(void)
{
    s_status_divider++;
    if (s_status_divider >= APP_GCODE_STATUS_PERIOD_MS) {
        s_status_divider = 0;
        s_status_due = 1;
    }
    if (s_count > 0u && s_start_wait_ms < APP_GCODE_BLEND_START_DELAY_MS) {
        s_start_wait_ms++;
    }
}

bool GcodeStream_TryProcessLine(const char *line)
{
    if (line == 0 || !starts_gcode(line)) {
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
        s_hold = 1;
        GcodeStream_Clear();
        BinaryTraj_Stop();
        MotionPlanner_Stop();
        return true;
    }
    if (line[0] == '~') {
        s_hold = 0;
        return true;
    }
    if ((unsigned char)line[0] == 0x18u) {
        GcodeStream_Clear();
        BinaryTraj_Stop();
        s_hold = 0;
        MotionPlanner_Stop();
        Stepper_ClearError();
        sync_position_from_stepper();
        send_ok_for_line(line);
        return true;
    }
    if (line[0] == '$') {
        long ppr1 = 0;
        long ppr2 = 0;
        if (sscanf(line, "$100=%ld $101=%ld", &ppr1, &ppr2) == 2 ||
            sscanf(line, "$100=%ld", &ppr1) == 1) {
            if (ppr2 <= 0) {
                ppr2 = ppr1;
            }
            if (ppr1 <= 0 || ppr2 <= 0) {
                send_error(4);
            } else if (Stepper_IsBusy() || GcodeStream_PlannerCount() > 0u) {
                send_error(8);
            } else {
                AppParams *p = AppParams_Mutable();
                p->pulses_per_rev[0] = (int32_t)ppr1;
                p->pulses_per_rev[1] = (int32_t)ppr2;
                sync_position_from_stepper();
                send_ok_for_line(line);
            }
            return true;
        }
        if (strcmp(line, "$$") == 0) {
            const AppParams *p = AppParams_Get();
            SerialDma_SendFormat("$100=%ld\n$101=%ld\n$110=%ld\n$111=%ld\nok\n",
                                 (long)p->pulses_per_rev[0],
                                 (long)p->pulses_per_rev[1],
                                 (long)APP_CONTROL_HZ,
                                 (long)APP_GCODE_PLANNER_BLOCKS);
            return true;
        }
        char cmd = (char)toupper((unsigned char)line[1]);
        if (cmd == 'X') {
            Stepper_ClearError();
            send_ok_for_line(line);
        } else if (cmd == 'H') {
            const char *home_args = line;
            while (*home_args != '\0' && !isspace((unsigned char)*home_args)) {
                home_args++;
            }
            while (*home_args != '\0' && isspace((unsigned char)*home_args)) {
                home_args++;
            }
            bool simulated = (toupper((unsigned char)home_args[0]) == 'S');
            if (HomeController_Start(simulated)) {
                send_ok_for_line(line);
            } else {
                send_error(5);
            }
        } else if (cmd == 'G') {
            SerialDma_SendFormat("[GC:G%u G%u G%u F%ld]\n",
                                 (unsigned int)s_gc.motion_mode,
                                 s_gc.absolute ? 90u : 91u,
                                 s_gc.mm_units ? 21u : 20u,
                                 (long)s_gc.feed_mm_min);
            send_ok_for_line(line);
        } else {
            send_error(3);
        }
        return true;
    }

    if ((toupper((unsigned char)line[0]) == 'M') &&
        (line[1] == '1') && (line[2] == '1') && (line[3] == '2')) {
        GcodeStream_Clear();
        BinaryTraj_Stop();
        HomeController_Stop();
        Stepper_EStopAll();
        send_ok_for_line(line);
        return true;
    }

    if (s_pending_valid) {
        send_error(8);
        return true;
    }

    if (buffer_full()) {
        strncpy(s_pending, line, sizeof(s_pending) - 1u);
        s_pending[sizeof(s_pending) - 1u] = '\0';
        s_pending_valid = 1;
        return true;
    }

    return process_block(line, 1);
}
