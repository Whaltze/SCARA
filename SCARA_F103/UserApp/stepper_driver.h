#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#define STEPPER_ERR_SOFT_LIMIT 0x00000001u
#define STEPPER_ERR_ESTOP 0x00000002u
#define STEPPER_ERR_COMM_TIMEOUT 0x00000004u
#define STEPPER_ERR_DISABLED 0x00000008u
#define STEPPER_ERR_INVALID_ARG 0x00000010u

typedef enum {
    STEPPER_AXIS_1 = 0,
    STEPPER_AXIS_2 = 1
} StepperAxis;

typedef enum {
    STEPPER_MODE_IDLE = 0,
    STEPPER_MODE_SPEED,
    STEPPER_MODE_MOVE,
    STEPPER_MODE_STOPPING,
    STEPPER_MODE_ESTOP
} StepperMode;

typedef struct {
    bool enabled;
    bool running;
    int8_t dir;
    StepperMode mode;
    int32_t current_pps;
    int32_t target_pps;
    int32_t exit_pps;
    int32_t accel_pps_s;
    int32_t max_pps;
    int64_t position_pulse;
    int64_t target_position_pulse;
    int64_t remaining_pulse;
    uint32_t error;
} StepperAxisState;

typedef struct {
    StepperAxisState axis[2];
    uint32_t tick_ms;
} StepperState;

void Stepper_Init(void);
void Stepper_Enable(StepperAxis axis, bool enable);
void Stepper_EnableAll(bool enable);
void Stepper_SetDir(StepperAxis axis, int8_t dir);
void Stepper_SetPps(StepperAxis axis, int32_t pps);
void Stepper_SetAccel(StepperAxis axis, int32_t accel_pps_s);
bool Stepper_MoveRel(int64_t delta1, int64_t delta2, int32_t v1, int32_t v2);
bool Stepper_MoveAbs(int64_t pos1, int64_t pos2, int32_t v1, int32_t v2);
bool Stepper_MoveAbsBlend(int64_t pos1, int64_t pos2, int32_t v1, int32_t v2, int32_t exit1, int32_t exit2);
bool Stepper_MoveAbsTicks(int64_t pos1, int64_t pos2, uint32_t duration_ticks);
void Stepper_Stop(StepperAxis axis);
void Stepper_StopAll(void);
void Stepper_EStopAll(void);
void Stepper_ClearError(void);
void Stepper_SetErrorAll(uint32_t error_bits);
void Stepper_SetPosition(StepperAxis axis, int64_t position_pulse);
void Stepper_Zero(void);
void Stepper_Tick10kHz(void);
bool Stepper_IsBusy(void);
bool Stepper_CanAcceptMove(void);
bool Stepper_CanQueueTimedSegment(void);
uint8_t Stepper_TimedSegmentCount(void);
uint8_t Stepper_TimedSegmentFree(void);
bool Stepper_TargetsAllowed(int64_t pos1, int64_t pos2);
void Stepper_GetStateSnapshot(StepperState *out);
const char *Stepper_ModeName(StepperMode mode);

#endif
