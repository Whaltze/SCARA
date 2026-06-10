#ifndef SCARA_KINEMATICS_H
#define SCARA_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t theta1_mrad;
    int32_t theta2_mrad;
} ScaraJoint;

typedef struct {
    int32_t x_um;
    int32_t y_um;
} ScaraPose;

void ScaraKinematics_Init(void);
bool ScaraKinematics_JointToPulse(int32_t theta1_mrad, int32_t theta2_mrad,
                                  int64_t *pulse1, int64_t *pulse2);
bool ScaraKinematics_PulseToJoint(int64_t pulse1, int64_t pulse2, ScaraJoint *joint);
bool ScaraKinematics_InverseUm(int32_t x_um, int32_t y_um, ScaraJoint *joint);
bool ScaraKinematics_Forward(const ScaraJoint *joint, ScaraPose *pose);
bool ScaraKinematics_InverseUmToPulse(int32_t x_um, int32_t y_um,
                                      int64_t *pulse1, int64_t *pulse2);
bool ScaraKinematics_PulseToPose(int64_t pulse1, int64_t pulse2, ScaraPose *pose);

#endif
