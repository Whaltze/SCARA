#ifndef LASER_CONTROL_H
#define LASER_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool armed;
    bool relay_ready;
    bool marking;
    bool task_started;
    bool pwm_ready;
    bool boot_ready;
    uint16_t power_permille;
} LaserControlState;

void LaserControl_EarlySafeOutput(void);
void LaserControl_Init(void);
bool LaserControl_SetPowerPermille(uint16_t power_permille);
bool LaserControl_Arm(void);
void LaserControl_Disarm(void);
bool LaserControl_CanMove(void);
void LaserControl_BeginSegment(bool marking, bool prep);
void LaserControl_Tick1kHz(bool controller_idle);
void LaserControl_GetState(LaserControlState *out);

#endif
