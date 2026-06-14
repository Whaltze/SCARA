#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTION_LASER_OFF = 0,
    MOTION_LASER_CONSTANT = 1,
    MOTION_LASER_DYNAMIC = 2,
    MOTION_LASER_PREP = 3
} MotionLaserMode;

typedef struct {
    uint8_t planner_count;
    uint8_t planner_free;
    uint8_t segment_count;
    uint8_t segment_free;
    uint8_t segment_low_water;
    uint32_t segment_underrun_count;
    uint32_t preparation_fault_count;
    uint32_t rate_limited_segment_count;
    uint32_t max_refill_gap_ms;
    uint32_t prepared_segments;
    uint32_t completed_blocks;
} MotionPlannerSnapshot;

void MotionPlanner_Init(void);
void MotionPlanner_Loop(void);
void MotionPlanner_Tick1kHz(void);
void MotionPlanner_Clear(void);
void MotionPlanner_Stop(void);
void MotionPlanner_SetHold(bool hold);
bool MotionPlanner_IsBusy(void);

bool MotionPlanner_PlanLine(int32_t start_x_um,
                            int32_t start_y_um,
                            int32_t end_x_um,
                            int32_t end_y_um,
                            int32_t feed_mm_min,
                            bool rapid,
                            bool exact_stop,
                            MotionLaserMode laser_mode);
bool MotionPlanner_PlanArc(int32_t start_x_um,
                           int32_t start_y_um,
                           int32_t end_x_um,
                           int32_t end_y_um,
                           int32_t center_x_um,
                           int32_t center_y_um,
                           bool clockwise,
                           int32_t feed_mm_min,
                           bool exact_stop,
                           MotionLaserMode laser_mode);
bool MotionPlanner_PlanDwell(uint32_t duration_ms, MotionLaserMode laser_mode);

uint8_t MotionPlanner_Free(void);
uint8_t MotionPlanner_Count(void);
void MotionPlanner_GetSnapshot(MotionPlannerSnapshot *out);
void MotionPlanner_SetAcceleration(float accel_mm_s2);
void MotionPlanner_SetJunctionDeviation(float junction_deviation_mm);
void MotionPlanner_SetArcTolerance(float arc_tolerance_mm);
void MotionPlanner_SetMaxFeed(float max_feed_mm_min);
void MotionPlanner_SetLaserPower(uint16_t power_permille);
float MotionPlanner_GetAcceleration(void);
float MotionPlanner_GetJunctionDeviation(void);
float MotionPlanner_GetArcTolerance(void);
float MotionPlanner_GetMaxFeed(void);

#endif
