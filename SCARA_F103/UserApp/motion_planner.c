#include "motion_planner.h"

/*
 * SCARA adaptation of Grbl's planner/step-segment ownership model.
 *
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 * Copyright (c) 2011-2019 Sungeun K. Jeon
 * SCARA/STM32 adaptation copyright (c) 2026
 *
 * Grbl is licensed under GPLv3. This adaptation keeps the reverse/forward
 * look-ahead model and the planner-to-step-segment split, while planner blocks
 * remain Cartesian and are converted to five-bar joint pulses during segment
 * preparation.
 */

#include "app_config.h"
#include "scara_kinematics.h"
#include "stepper_driver.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define MP_PI_F 3.14159265358979323846f
#define MP_TWO_PI_F (2.0f * MP_PI_F)
#define MP_FLAG_RAPID 0x01u
#define MP_FLAG_EXACT_STOP 0x02u
#define MP_FLAG_ARC 0x04u
#define MP_FLAG_DWELL 0x08u

typedef struct {
    int32_t start_x_um;
    int32_t start_y_um;
    int32_t end_x_um;
    int32_t end_y_um;
    int32_t center_x_um;
    int32_t center_y_um;
    float length_mm;
    float nominal_speed_mm_s;
    float acceleration_mm_s2;
    float max_entry_speed_sqr;
    float entry_speed_sqr;
    float start_angle;
    float delta_angle;
    float joint_start_slope1;
    float joint_start_slope2;
    float joint_end_slope1;
    float joint_end_slope2;
    uint32_t dwell_ms;
    uint8_t flags;
    MotionLaserMode laser_mode;
} MotionBlock;

typedef struct {
    bool active;
    uint8_t block_index;
    float distance_mm;
    float speed_mm_s;
    uint32_t dwell_remaining_ms;
} MotionPrep;

static MotionBlock s_blocks[APP_GCODE_PLANNER_BLOCKS];
static uint8_t s_head;
static uint8_t s_tail;
static uint8_t s_count;
static MotionPrep s_prep;
static float s_accel_mm_s2 = APP_GRBL_ACCEL_MM_S2;
static float s_junction_deviation_mm = APP_GRBL_JUNCTION_DEVIATION_MM;
static float s_arc_tolerance_mm = APP_GRBL_ARC_TOLERANCE_MM;
static float s_max_feed_mm_min = APP_GRBL_MAX_FEED_MM_MIN;
static uint16_t s_laser_power_permille = APP_LASER_POWER_DEFAULT_PERMILLE;
static uint8_t s_segment_low_water;
static uint32_t s_segment_underrun_count;
static uint32_t s_preparation_fault_count;
static uint32_t s_rate_limited_segment_count;
static uint32_t s_refill_gap_ms;
static uint32_t s_max_refill_gap_ms;
static uint32_t s_prepared_segments;
static uint32_t s_completed_blocks;
static bool s_stream_started;
static bool s_hold;
static uint16_t s_prefill_wait_ms;
static int64_t s_last_segment_p1;
static int64_t s_last_segment_p2;
static bool s_last_segment_valid;
static bool s_preparation_fault_latched;

static uint8_t next_index(uint8_t index)
{
    index++;
    return index >= APP_GCODE_PLANNER_BLOCKS ? 0u : index;
}

static uint8_t prev_index(uint8_t index)
{
    return index == 0u ? (APP_GCODE_PLANNER_BLOCKS - 1u) : (uint8_t)(index - 1u);
}

static float clamp_f(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static float sqr_f(float value)
{
    return value * value;
}

static void point_at(const MotionBlock *block, float distance_mm, int32_t *x_um, int32_t *y_um);

static uint32_t final_segment_ticks(float distance_mm,
                                    float start_speed_mm_s,
                                    float end_speed_mm_s,
                                    uint32_t max_ticks)
{
    float average_speed = 0.5f * (start_speed_mm_s + end_speed_mm_s);
    if (distance_mm <= 0.000001f || average_speed <= 0.000001f) {
        return max_ticks;
    }
    uint32_t ticks = (uint32_t)lroundf(distance_mm * (float)APP_CONTROL_HZ / average_speed);
    if (ticks < 1u) {
        ticks = 1u;
    }
    return ticks < max_ticks ? ticks : max_ticks;
}

static void block_tangent(const MotionBlock *block, bool at_end, float *tx, float *ty)
{
    if ((block->flags & MP_FLAG_ARC) != 0u) {
        float angle = at_end ? block->start_angle + block->delta_angle : block->start_angle;
        float sign = block->delta_angle >= 0.0f ? 1.0f : -1.0f;
        *tx = -sinf(angle) * sign;
        *ty = cosf(angle) * sign;
        return;
    }

    float dx = (float)(block->end_x_um - block->start_x_um);
    float dy = (float)(block->end_y_um - block->start_y_um);
    float length = sqrtf(dx * dx + dy * dy);
    if (length <= 0.0f) {
        *tx = 1.0f;
        *ty = 0.0f;
    } else {
        *tx = dx / length;
        *ty = dy / length;
    }
}

static float junction_speed_sqr(const MotionBlock *previous, const MotionBlock *current)
{
    if ((previous->flags & (MP_FLAG_EXACT_STOP | MP_FLAG_DWELL)) != 0u ||
        (current->flags & (MP_FLAG_EXACT_STOP | MP_FLAG_DWELL)) != 0u ||
        previous->laser_mode != current->laser_mode) {
        return 0.0f;
    }

    float ptx;
    float pty;
    float ctx;
    float cty;
    block_tangent(previous, true, &ptx, &pty);
    block_tangent(current, false, &ctx, &cty);
    float dot = clamp_f(ptx * ctx + pty * cty, -1.0f, 1.0f);
    if (dot > 0.999999f) {
        float speed = previous->nominal_speed_mm_s < current->nominal_speed_mm_s
                          ? previous->nominal_speed_mm_s
                          : current->nominal_speed_mm_s;
        return sqr_f(speed);
    }
    if (dot <= -0.999999f) {
        return 0.0f;
    }

    /* Grbl centripetal junction-deviation model. */
    float sin_theta_d2 = sqrtf(0.5f * (1.0f + dot));
    float denom = 1.0f - sin_theta_d2;
    if (denom <= 0.000001f) {
        return sqr_f(current->nominal_speed_mm_s);
    }
    float accel = previous->acceleration_mm_s2 < current->acceleration_mm_s2
                      ? previous->acceleration_mm_s2
                      : current->acceleration_mm_s2;
    float limit = accel * s_junction_deviation_mm * sin_theta_d2 / denom;
    float nominal_limit = sqr_f(previous->nominal_speed_mm_s < current->nominal_speed_mm_s
                                    ? previous->nominal_speed_mm_s
                                    : current->nominal_speed_mm_s);
    if (limit > nominal_limit) {
        limit = nominal_limit;
    }

    /*
     * Grbl applies junction limits in the coordinates of the physical axes.
     * For a five-bar SCARA, a modest Cartesian turn can still reverse a joint.
     * Constrain the same junction-deviation model by the endpoint IK slopes so
     * the motors never receive an unplanned joint-space velocity corner.
     */
    float prev_joint_mag = hypotf(previous->joint_end_slope1, previous->joint_end_slope2);
    float curr_joint_mag = hypotf(current->joint_start_slope1, current->joint_start_slope2);
    if (prev_joint_mag > 0.000001f && curr_joint_mag > 0.000001f) {
        float joint_dot = clamp_f(
            (previous->joint_end_slope1 * current->joint_start_slope1 +
             previous->joint_end_slope2 * current->joint_start_slope2) /
                (prev_joint_mag * curr_joint_mag),
            -1.0f,
            1.0f);
        if (joint_dot <= -0.999999f) {
            return 0.0f;
        }
        if (joint_dot < 0.999999f) {
            float joint_sin_theta_d2 = sqrtf(0.5f * (1.0f + joint_dot));
            float joint_denom = 1.0f - joint_sin_theta_d2;
            if (joint_denom > 0.000001f) {
                float pulse_per_mm = prev_joint_mag > curr_joint_mag ? prev_joint_mag : curr_joint_mag;
                float joint_deviation_pulse =
                    s_junction_deviation_mm *
                    (prev_joint_mag < curr_joint_mag ? prev_joint_mag : curr_joint_mag);
                float joint_pps_limit_sqr =
                    (0.90f * (float)APP_ACCEL_MAX) *
                    joint_deviation_pulse *
                    joint_sin_theta_d2 /
                    joint_denom;
                float joint_cartesian_limit = joint_pps_limit_sqr / sqr_f(pulse_per_mm);
                if (limit > joint_cartesian_limit) {
                    limit = joint_cartesian_limit;
                }
            }
        }
    }
    return limit;
}

static void planner_recalculate(void)
{
    if (s_count == 0u) {
        return;
    }

    uint8_t index = s_tail;
    MotionBlock *previous = NULL;
    for (uint8_t i = 0u; i < s_count; ++i) {
        MotionBlock *block = &s_blocks[index];
        if (i == 0u) {
            if (s_prep.active && s_prep.block_index == index) {
                block->max_entry_speed_sqr = sqr_f(s_prep.speed_mm_s);
            } else if (s_stream_started) {
                /*
                 * The predecessor can already live in the step-segment FIFO.
                 * Its planned junction speed is then stored on this block and
                 * must remain the fixed entry anchor during later replans.
                 */
                block->max_entry_speed_sqr = block->entry_speed_sqr;
            } else {
                block->max_entry_speed_sqr = 0.0f;
            }
        } else {
            block->max_entry_speed_sqr = junction_speed_sqr(previous, block);
        }
        block->entry_speed_sqr = block->max_entry_speed_sqr;
        previous = block;
        index = next_index(index);
    }

    float next_entry_sqr = 0.0f;
    index = prev_index(s_head);
    for (uint8_t i = 0u; i < s_count; ++i) {
        MotionBlock *block = &s_blocks[index];
        float allowed = next_entry_sqr + 2.0f * block->acceleration_mm_s2 * block->length_mm;
        if (block->entry_speed_sqr > allowed) {
            block->entry_speed_sqr = allowed;
        }
        next_entry_sqr = block->entry_speed_sqr;
        index = prev_index(index);
    }

    index = s_tail;
    for (uint8_t i = 1u; i < s_count; ++i) {
        MotionBlock *previous_block = &s_blocks[index];
        index = next_index(index);
        MotionBlock *block = &s_blocks[index];
        float allowed = previous_block->entry_speed_sqr +
                        2.0f * previous_block->acceleration_mm_s2 * previous_block->length_mm;
        if (block->entry_speed_sqr > allowed) {
            block->entry_speed_sqr = allowed;
        }
    }
}

static bool validate_and_limit_path(MotionBlock *block)
{
    uint32_t samples = 4u;
    if (block->length_mm > 1.0f) {
        samples = (uint32_t)ceilf(block->length_mm / 2.0f);
        if (samples > 64u) {
            samples = 64u;
        }
    }
    int32_t x_um;
    int32_t y_um;
    int64_t previous_p1;
    int64_t previous_p2;
    point_at(block, 0.0f, &x_um, &y_um);
    if (!ScaraKinematics_InverseUmToPulse(x_um, y_um, &previous_p1, &previous_p2) ||
        !Stepper_TargetsAllowed(previous_p1, previous_p2)) {
        return false;
    }

    float sample_mm = block->length_mm / (float)samples;
    float previous_slope1 = 0.0f;
    float previous_slope2 = 0.0f;
    bool previous_slope_valid = false;
    for (uint32_t i = 1u; i <= samples; ++i) {
        float sample_distance = i == samples ? block->length_mm : sample_mm * (float)i;
        point_at(block, sample_distance, &x_um, &y_um);
        int64_t p1;
        int64_t p2;
        if (!ScaraKinematics_InverseUmToPulse(x_um, y_um, &p1, &p2) ||
            !Stepper_TargetsAllowed(p1, p2)) {
            return false;
        }
        float slope1 = (float)(p1 - previous_p1) / sample_mm;
        float slope2 = (float)(p2 - previous_p2) / sample_mm;
        if (i == 1u) {
            block->joint_start_slope1 = slope1;
            block->joint_start_slope2 = slope2;
        }
        block->joint_end_slope1 = slope1;
        block->joint_end_slope2 = slope2;
        float max_slope = fabsf(slope1) > fabsf(slope2) ? fabsf(slope1) : fabsf(slope2);
        if (max_slope > 0.000001f) {
            /*
             * Grbl constrains each planner block before look-ahead. For SCARA,
             * Cartesian feed and acceleration must first be mapped through the
             * local IK slope so segment preparation never has to reject a
             * planned block to protect the joint pulse rate.
             */
            float feed_limit = (0.90f * (float)APP_MAX_PPS_DEFAULT) / max_slope;
            float accel_limit = (0.90f * (float)APP_ACCEL_MAX) / max_slope;
            if (block->nominal_speed_mm_s > feed_limit) {
                block->nominal_speed_mm_s = feed_limit;
            }
            if (block->acceleration_mm_s2 > accel_limit) {
                block->acceleration_mm_s2 = accel_limit;
            }
        }
        if (previous_slope_valid) {
            float curvature1 = fabsf(slope1 - previous_slope1) / sample_mm;
            float curvature2 = fabsf(slope2 - previous_slope2) / sample_mm;
            float max_curvature = curvature1 > curvature2 ? curvature1 : curvature2;
            if (max_curvature > 0.000001f) {
                float curve_speed_limit = sqrtf((0.90f * (float)APP_ACCEL_MAX) / max_curvature);
                if (block->nominal_speed_mm_s > curve_speed_limit) {
                    block->nominal_speed_mm_s = curve_speed_limit;
                }
            }
        }
        previous_p1 = p1;
        previous_p2 = p2;
        previous_slope1 = slope1;
        previous_slope2 = slope2;
        previous_slope_valid = true;
    }
    block->nominal_speed_mm_s = clamp_f(block->nominal_speed_mm_s, 0.01f, s_max_feed_mm_min / 60.0f);
    block->acceleration_mm_s2 = clamp_f(block->acceleration_mm_s2, 1.0f, APP_GRBL_MAX_ACCEL_MM_S2);
    return true;
}

static bool enqueue_block(const MotionBlock *source)
{
    if (s_count >= APP_GCODE_PLANNER_BLOCKS) {
        return false;
    }
    s_blocks[s_head] = *source;
    s_head = next_index(s_head);
    s_count++;
    planner_recalculate();
    return true;
}

void MotionPlanner_Init(void)
{
    memset(s_blocks, 0, sizeof(s_blocks));
    memset(&s_prep, 0, sizeof(s_prep));
    s_head = 0u;
    s_tail = 0u;
    s_count = 0u;
    s_segment_low_water = APP_STEPPER_TIMED_SEGMENTS;
    s_segment_underrun_count = 0u;
    s_preparation_fault_count = 0u;
    s_rate_limited_segment_count = 0u;
    s_refill_gap_ms = 0u;
    s_max_refill_gap_ms = 0u;
    s_prepared_segments = 0u;
    s_completed_blocks = 0u;
    s_stream_started = false;
    s_hold = false;
    s_prefill_wait_ms = 0u;
    s_refill_gap_ms = 0u;
    s_last_segment_valid = false;
    s_preparation_fault_latched = false;
}

void MotionPlanner_Clear(void)
{
    s_head = 0u;
    s_tail = 0u;
    s_count = 0u;
    memset(&s_prep, 0, sizeof(s_prep));
    s_stream_started = false;
    s_hold = false;
    s_prefill_wait_ms = 0u;
    s_last_segment_valid = false;
    s_preparation_fault_latched = false;
}

void MotionPlanner_Stop(void)
{
    MotionPlanner_Clear();
    Stepper_StopAll();
}

static void preparation_fault_stop(void)
{
    s_preparation_fault_count++;
    MotionPlanner_Stop();
    s_preparation_fault_latched = true;
}

void MotionPlanner_SetHold(bool hold)
{
    s_hold = hold;
}

bool MotionPlanner_PlanLine(int32_t start_x_um,
                            int32_t start_y_um,
                            int32_t end_x_um,
                            int32_t end_y_um,
                            int32_t feed_mm_min,
                            bool rapid,
                            bool exact_stop,
                            MotionLaserMode laser_mode)
{
    if (s_preparation_fault_latched) {
        return false;
    }
    if (s_count >= APP_GCODE_PLANNER_BLOCKS) {
        return false;
    }
    int64_t dx = (int64_t)end_x_um - start_x_um;
    int64_t dy = (int64_t)end_y_um - start_y_um;
    float length_mm = sqrtf((float)(dx * dx + dy * dy)) / 1000.0f;
    if (length_mm < 0.0005f) {
        return true;
    }

    MotionBlock block;
    memset(&block, 0, sizeof(block));
    block.start_x_um = start_x_um;
    block.start_y_um = start_y_um;
    block.end_x_um = end_x_um;
    block.end_y_um = end_y_um;
    block.length_mm = length_mm;
    float max_feed_mm_s = s_max_feed_mm_min / 60.0f;
    block.nominal_speed_mm_s = rapid
                                   ? clamp_f(APP_GRBL_RAPID_MM_MIN / 60.0f, 0.01f, max_feed_mm_s)
                                   : clamp_f((float)feed_mm_min / 60.0f, 0.01f, max_feed_mm_s);
    block.acceleration_mm_s2 = s_accel_mm_s2;
    block.flags = (rapid ? MP_FLAG_RAPID : 0u) | (exact_stop ? MP_FLAG_EXACT_STOP : 0u);
    block.laser_mode = rapid ? MOTION_LASER_OFF : laser_mode;
    if (!validate_and_limit_path(&block)) {
        return false;
    }
    return enqueue_block(&block);
}

bool MotionPlanner_PlanArc(int32_t start_x_um,
                           int32_t start_y_um,
                           int32_t end_x_um,
                           int32_t end_y_um,
                           int32_t center_x_um,
                           int32_t center_y_um,
                           bool clockwise,
                           int32_t feed_mm_min,
                           bool exact_stop,
                           MotionLaserMode laser_mode)
{
    if (s_preparation_fault_latched) {
        return false;
    }
    if (s_count >= APP_GCODE_PLANNER_BLOCKS) {
        return false;
    }
    float sx = (float)(start_x_um - center_x_um);
    float sy = (float)(start_y_um - center_y_um);
    float ex = (float)(end_x_um - center_x_um);
    float ey = (float)(end_y_um - center_y_um);
    float radius = hypotf(sx, sy);
    if (radius < 1.0f || fabsf(radius - hypotf(ex, ey)) > (s_arc_tolerance_mm * 1000.0f + 2.0f)) {
        return false;
    }
    float start_angle = atan2f(sy, sx);
    float end_angle = atan2f(ey, ex);
    float delta = end_angle - start_angle;
    if (clockwise) {
        while (delta >= 0.0f) {
            delta -= MP_TWO_PI_F;
        }
    } else {
        while (delta <= 0.0f) {
            delta += MP_TWO_PI_F;
        }
    }

    MotionBlock block;
    memset(&block, 0, sizeof(block));
    block.start_x_um = start_x_um;
    block.start_y_um = start_y_um;
    block.end_x_um = end_x_um;
    block.end_y_um = end_y_um;
    block.center_x_um = center_x_um;
    block.center_y_um = center_y_um;
    block.start_angle = start_angle;
    block.delta_angle = delta;
    block.length_mm = fabsf(delta) * radius / 1000.0f;
    block.nominal_speed_mm_s = clamp_f((float)feed_mm_min / 60.0f, 0.01f, s_max_feed_mm_min / 60.0f);
    block.acceleration_mm_s2 = s_accel_mm_s2;
    block.flags = MP_FLAG_ARC | (exact_stop ? MP_FLAG_EXACT_STOP : 0u);
    block.laser_mode = laser_mode;
    if (!validate_and_limit_path(&block)) {
        return false;
    }
    return enqueue_block(&block);
}

bool MotionPlanner_PlanDwell(uint32_t duration_ms, MotionLaserMode laser_mode)
{
    if (s_preparation_fault_latched) {
        return false;
    }
    if (duration_ms == 0u) {
        return true;
    }
    MotionBlock block;
    memset(&block, 0, sizeof(block));
    block.flags = MP_FLAG_DWELL | MP_FLAG_EXACT_STOP;
    block.dwell_ms = duration_ms;
    block.laser_mode = laser_mode;
    return enqueue_block(&block);
}

static void point_at(const MotionBlock *block, float distance_mm, int32_t *x_um, int32_t *y_um)
{
    if (distance_mm <= 0.0f) {
        *x_um = block->start_x_um;
        *y_um = block->start_y_um;
        return;
    }
    if (distance_mm >= block->length_mm) {
        *x_um = block->end_x_um;
        *y_um = block->end_y_um;
        return;
    }
    float ratio = block->length_mm <= 0.0f ? 1.0f : clamp_f(distance_mm / block->length_mm, 0.0f, 1.0f);
    if ((block->flags & MP_FLAG_ARC) != 0u) {
        float radius_um = hypotf((float)(block->start_x_um - block->center_x_um),
                                 (float)(block->start_y_um - block->center_y_um));
        float angle = block->start_angle + block->delta_angle * ratio;
        *x_um = block->center_x_um + (int32_t)lroundf(radius_um * cosf(angle));
        *y_um = block->center_y_um + (int32_t)lroundf(radius_um * sinf(angle));
    } else {
        *x_um = block->start_x_um +
                (int32_t)lroundf((float)(block->end_x_um - block->start_x_um) * ratio);
        *y_um = block->start_y_um +
                (int32_t)lroundf((float)(block->end_y_um - block->start_y_um) * ratio);
    }
}

static bool queue_segment(int64_t p1,
                          int64_t p2,
                          uint32_t ticks,
                          MotionLaserMode laser_mode,
                          float speed_mm_s,
                          float nominal_speed_mm_s)
{
    if (!s_last_segment_valid) {
        StepperState state;
        Stepper_GetStateSnapshot(&state);
        s_last_segment_p1 = state.axis[0].target_position_pulse;
        s_last_segment_p2 = state.axis[1].target_position_pulse;
        s_last_segment_valid = true;
    }
    uint64_t d1 = (uint64_t)(p1 >= s_last_segment_p1 ? p1 - s_last_segment_p1 : s_last_segment_p1 - p1);
    uint64_t d2 = (uint64_t)(p2 >= s_last_segment_p2 ? p2 - s_last_segment_p2 : s_last_segment_p2 - p2);
    uint32_t events = (uint32_t)(d1 > d2 ? d1 : d2);
    if (ticks < events) {
        return false;
    }
    uint16_t power = 0u;
    if (laser_mode == MOTION_LASER_DYNAMIC && nominal_speed_mm_s > 0.0f) {
        float ratio = clamp_f(speed_mm_s / nominal_speed_mm_s, 0.0f, 1.0f);
        power = (uint16_t)lroundf((float)s_laser_power_permille * ratio);
        if (power > 0u && power < APP_LASER_POWER_MIN_PERMILLE) {
            power = APP_LASER_POWER_MIN_PERMILLE;
        }
    }
    bool mark = laser_mode == MOTION_LASER_CONSTANT ||
                (laser_mode == MOTION_LASER_DYNAMIC && power > 0u);
    bool prep = laser_mode == MOTION_LASER_PREP;
    if (!Stepper_MoveAbsTicksLaserPower(p1, p2, ticks, mark, prep, power)) {
        return false;
    }
    s_last_segment_p1 = p1;
    s_last_segment_p2 = p2;
    if (s_refill_gap_ms > s_max_refill_gap_ms) {
        s_max_refill_gap_ms = s_refill_gap_ms;
    }
    s_refill_gap_ms = 0u;
    s_prepared_segments++;
    uint8_t count = Stepper_TimedSegmentCount();
    if (count < s_segment_low_water) {
        s_segment_low_water = count;
    }
    return true;
}

static void complete_prepared_block(void)
{
    s_tail = next_index(s_tail);
    if (s_count > 0u) {
        s_count--;
    }
    s_completed_blocks++;
    memset(&s_prep, 0, sizeof(s_prep));
}

void MotionPlanner_Loop(void)
{
    if (s_hold && (!s_prep.active || s_prep.speed_mm_s <= 0.0001f)) {
        return;
    }
    if (!s_stream_started && s_count > 0u) {
        MotionBlock *first = &s_blocks[s_tail];
        if ((first->flags & (MP_FLAG_EXACT_STOP | MP_FLAG_DWELL)) == 0u &&
            s_count < APP_GCODE_BLEND_MIN_BLOCKS &&
            s_prefill_wait_ms < APP_GCODE_BLEND_START_DELAY_MS) {
            return;
        }
    }
    while (Stepper_TimedSegmentFree() > 0u && s_count > 0u) {
        MotionBlock *block = &s_blocks[s_tail];
        if (s_stream_started &&
            s_count == 1u &&
            (block->flags & (MP_FLAG_EXACT_STOP | MP_FLAG_DWELL)) == 0u &&
            Stepper_TimedSegmentCount() >= APP_GCODE_TAIL_HOLDBACK_SEGMENTS) {
            /*
             * Grbl keeps unchecked planner distance available for incoming
             * short blocks. Do not eagerly consume the final unknown-successor
             * block while the step FIFO still has enough execution margin.
             */
            break;
        }
        if (!s_prep.active) {
            s_prep.active = true;
            s_prep.block_index = s_tail;
            s_prep.distance_mm = 0.0f;
            s_prep.speed_mm_s = sqrtf(block->entry_speed_sqr);
            s_prep.dwell_remaining_ms = block->dwell_ms;
        }

        if ((block->flags & MP_FLAG_DWELL) != 0u) {
            StepperState state;
            Stepper_GetStateSnapshot(&state);
            /*
             * A dwell is prepared behind already-queued motion. The stepper
             * snapshot only exposes the active segment target, which can lag
             * many segments behind the planner endpoint. Hold the last
             * prepared endpoint so a stroke barrier never queues a rewind.
             */
            int64_t dwell_p1 = s_last_segment_valid
                                   ? s_last_segment_p1
                                   : state.axis[0].target_position_pulse;
            int64_t dwell_p2 = s_last_segment_valid
                                   ? s_last_segment_p2
                                   : state.axis[1].target_position_pulse;
            uint32_t slice_ms = s_prep.dwell_remaining_ms > APP_GRBL_SEGMENT_MS
                                    ? APP_GRBL_SEGMENT_MS
                                    : s_prep.dwell_remaining_ms;
            uint32_t ticks = slice_ms * APP_CONTROL_HZ / 1000u;
            if (!queue_segment(dwell_p1,
                               dwell_p2,
                               ticks,
                               block->laser_mode,
                               0.0f,
                               0.0f)) {
                break;
            }
            s_prep.dwell_remaining_ms -= slice_ms;
            if (s_prep.dwell_remaining_ms == 0u) {
                complete_prepared_block();
            }
            continue;
        }

        float remaining = block->length_mm - s_prep.distance_mm;
        if (remaining <= 0.000001f) {
            complete_prepared_block();
            continue;
        }
        float exit_speed = 0.0f;
        if (s_count > 1u && (block->flags & MP_FLAG_EXACT_STOP) == 0u) {
            exit_speed = sqrtf(s_blocks[next_index(s_tail)].entry_speed_sqr);
        }
        float dt = (float)APP_GRBL_SEGMENT_MS / 1000.0f;
        float stop_limited = sqrtf(sqr_f(exit_speed) + 2.0f * block->acceleration_mm_s2 * remaining);
        float desired = s_hold
                            ? 0.0f
                            : (block->nominal_speed_mm_s < stop_limited ? block->nominal_speed_mm_s : stop_limited);
        float delta_v = block->acceleration_mm_s2 * dt;
        float next_speed = s_prep.speed_mm_s;
        if (next_speed < desired) {
            next_speed += delta_v;
            if (next_speed > desired) {
                next_speed = desired;
            }
        } else if (next_speed > desired) {
            next_speed -= delta_v;
            if (next_speed < desired) {
                next_speed = desired;
            }
        }
        float distance = 0.5f * (s_prep.speed_mm_s + next_speed) * dt;
        if (distance <= 0.000001f) {
            distance = remaining < 0.0001f ? remaining : 0.0001f;
        }
        bool final_segment = distance >= remaining;
        if (final_segment) {
            distance = remaining;
        }
        float next_distance = s_prep.distance_mm + distance;
        if (final_segment) {
            next_distance = block->length_mm;
        }
        int32_t x_um;
        int32_t y_um;
        int64_t p1;
        int64_t p2;
        StepperState state;
        Stepper_GetStateSnapshot(&state);
        uint32_t max_ticks = APP_GRBL_SEGMENT_MS * APP_CONTROL_HZ / 1000u;
        uint32_t ticks = max_ticks;
        int64_t base1 = s_last_segment_valid ? s_last_segment_p1 : state.axis[0].target_position_pulse;
        int64_t base2 = s_last_segment_valid ? s_last_segment_p2 : state.axis[1].target_position_pulse;
        if (final_segment) {
            next_speed = exit_speed;
        }
        for (uint8_t attempt = 0u;; ++attempt) {
            ticks = final_segment
                        ? final_segment_ticks(distance, s_prep.speed_mm_s, next_speed, max_ticks)
                        : max_ticks;
            next_distance = final_segment ? block->length_mm : s_prep.distance_mm + distance;
            point_at(block, next_distance, &x_um, &y_um);
            if (!ScaraKinematics_InverseUmToPulse(x_um, y_um, &p1, &p2)) {
                preparation_fault_stop();
                return;
            }
            uint64_t d1 = (uint64_t)(p1 >= base1 ? p1 - base1 : base1 - p1);
            uint64_t d2 = (uint64_t)(p2 >= base2 ? p2 - base2 : base2 - p2);
            uint32_t events = (uint32_t)(d1 > d2 ? d1 : d2);
            if (events <= ticks) {
                break;
            }
            /*
             * Like Grbl's step-segment preparation, a planned block is never
             * discarded merely because one candidate segment is too dense.
             * Keep the block/prep state intact and shorten only this segment
             * until the fixed-rate DDA can represent every pulse event.
             */
            if (final_segment && ticks < max_ticks) {
                ticks = max_ticks;
                if (events <= ticks) {
                    break;
                }
            }
            if (attempt >= 24u || distance <= 0.00000001f) {
                preparation_fault_stop();
                return;
            }
            s_rate_limited_segment_count++;
            distance *= 0.95f * (float)ticks / (float)events;
            final_segment = false;
            next_speed = 2.0f * distance / ((float)max_ticks / (float)APP_CONTROL_HZ) -
                         s_prep.speed_mm_s;
            if (next_speed < 0.0f) {
                next_speed = 0.0f;
            }
        }
        if (!queue_segment(p1,
                           p2,
                           ticks,
                           block->laser_mode,
                           0.5f * (s_prep.speed_mm_s + next_speed),
                           block->nominal_speed_mm_s)) {
            break;
        }
        s_prep.distance_mm = next_distance;
        s_prep.speed_mm_s = next_speed;
        s_stream_started = true;
        if (final_segment) {
            complete_prepared_block();
        }
    }
}

void MotionPlanner_Tick1kHz(void)
{
    if (!s_stream_started && s_count > 0u && s_prefill_wait_ms < APP_GCODE_BLEND_START_DELAY_MS) {
        s_prefill_wait_ms++;
    }
    if (!s_hold && s_stream_started && s_count > 0u && !Stepper_IsBusy()) {
        s_segment_underrun_count++;
    }
    if (s_stream_started && s_count > 0u && Stepper_TimedSegmentFree() > 0u) {
        if (s_refill_gap_ms < UINT32_MAX) {
            s_refill_gap_ms++;
        }
    } else {
        s_refill_gap_ms = 0u;
    }
    if (s_count == 0u && !Stepper_IsBusy()) {
        s_stream_started = false;
        s_prefill_wait_ms = 0u;
        s_last_segment_valid = false;
    }
}

bool MotionPlanner_IsBusy(void)
{
    return s_count > 0u || Stepper_IsBusy();
}

uint8_t MotionPlanner_Free(void)
{
    return (uint8_t)(APP_GCODE_PLANNER_BLOCKS - s_count);
}

uint8_t MotionPlanner_Count(void)
{
    return s_count;
}

void MotionPlanner_GetSnapshot(MotionPlannerSnapshot *out)
{
    if (out == NULL) {
        return;
    }
    out->planner_count = s_count;
    out->planner_free = MotionPlanner_Free();
    out->segment_count = Stepper_TimedSegmentCount();
    out->segment_free = Stepper_TimedSegmentFree();
    out->segment_low_water = s_segment_low_water;
    out->segment_underrun_count = s_segment_underrun_count;
    out->preparation_fault_count = s_preparation_fault_count;
    out->rate_limited_segment_count = s_rate_limited_segment_count;
    out->max_refill_gap_ms = s_max_refill_gap_ms;
    out->prepared_segments = s_prepared_segments;
    out->completed_blocks = s_completed_blocks;
}

void MotionPlanner_SetAcceleration(float accel_mm_s2)
{
    s_accel_mm_s2 = clamp_f(accel_mm_s2, 1.0f, APP_GRBL_MAX_ACCEL_MM_S2);
}

void MotionPlanner_SetJunctionDeviation(float junction_deviation_mm)
{
    s_junction_deviation_mm = clamp_f(junction_deviation_mm, 0.001f, 5.0f);
}

void MotionPlanner_SetArcTolerance(float arc_tolerance_mm)
{
    s_arc_tolerance_mm = clamp_f(arc_tolerance_mm, 0.001f, 5.0f);
}

void MotionPlanner_SetMaxFeed(float max_feed_mm_min)
{
    s_max_feed_mm_min = clamp_f(max_feed_mm_min, 1.0f, APP_GRBL_MAX_FEED_MM_MIN);
}

void MotionPlanner_SetLaserPower(uint16_t power_permille)
{
    s_laser_power_permille = power_permille;
}

float MotionPlanner_GetAcceleration(void) { return s_accel_mm_s2; }
float MotionPlanner_GetJunctionDeviation(void) { return s_junction_deviation_mm; }
float MotionPlanner_GetArcTolerance(void) { return s_arc_tolerance_mm; }
float MotionPlanner_GetMaxFeed(void) { return s_max_feed_mm_min; }
