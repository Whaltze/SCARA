#include "scara_kinematics.h"

/* 五连杆并联 SCARA 运动学：课程版保留可读实现，后续可按实测连杆参数修正。 */

#include "app_params.h"
#include "app_config.h"

#include <math.h>
#include <stddef.h>

#define APP_PI_F 3.14159265358979323846f
#define APP_RAD_PER_REV_F (2.0f * APP_PI_F)
#define APP_EPSILON_F 0.0001f

void ScaraKinematics_Init(void)
{
}

static float clamp_f(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static bool joint_in_range(int32_t theta1_mrad, int32_t theta2_mrad)
{
#if APP_HOST_OWNS_LIMIT_CHECKS
    (void)theta1_mrad;
    (void)theta2_mrad;
    return true;
#else
    const AppParams *p = AppParams_Get();
    return theta1_mrad >= p->theta_min_mrad[0] &&
           theta1_mrad <= p->theta_max_mrad[0] &&
           theta2_mrad >= p->theta_min_mrad[1] &&
           theta2_mrad <= p->theta_max_mrad[1];
#endif
}

static bool inverse_um_radians(int32_t x_um, int32_t y_um, float *theta1, float *theta2)
{
    if (theta1 == NULL || theta2 == NULL) {
        return false;
    }

    const AppParams *p = AppParams_Get();
    const float half_base = (float)p->scara_base_um * 0.5f;
    const float l1 = (float)p->active_arm_um[0];
    const float l2 = (float)p->active_arm_um[1];
    const float l3 = (float)p->passive_arm_um[0];
    const float l4 = (float)p->passive_arm_um[1];
    const float x = (float)x_um;
    const float y = (float)y_um;
    const float dx1 = x + half_base;
    const float dx2 = x - half_base;
    const float r1 = sqrtf(dx1 * dx1 + y * y);
    const float r2 = sqrtf(dx2 * dx2 + y * y);

    if (r1 < APP_EPSILON_F || r2 < APP_EPSILON_F ||
        r1 > (l1 + l3) || r1 < fabsf(l1 - l3) ||
        r2 > (l2 + l4) || r2 < fabsf(l2 - l4)) {
        return false;
    }

    const float a1 = atan2f(y, dx1);
    const float a2 = atan2f(y, dx2);
    const float b1 = acosf(clamp_f((l1 * l1 + r1 * r1 - l3 * l3) / (2.0f * l1 * r1), -1.0f, 1.0f));
    const float b2 = acosf(clamp_f((l2 * l2 + r2 * r2 - l4 * l4) / (2.0f * l2 * r2), -1.0f, 1.0f));
    *theta1 = a1 + (float)p->ik_left_elbow_sign * b1;
    *theta2 = a2 + (float)p->ik_right_elbow_sign * b2;
    return joint_in_range((int32_t)lroundf(*theta1 * 1000.0f),
                          (int32_t)lroundf(*theta2 * 1000.0f));
}

static bool forward_radians(float theta1, float theta2, ScaraPose *pose)
{
    if (pose == NULL) {
        return false;
    }

    const AppParams *p = AppParams_Get();
    const float half_base = (float)p->scara_base_um * 0.5f;
    const float l1 = (float)p->active_arm_um[0];
    const float l2 = (float)p->active_arm_um[1];
    const float l3 = (float)p->passive_arm_um[0];
    const float l4 = (float)p->passive_arm_um[1];
    const float ex1 = -half_base + l1 * cosf(theta1);
    const float ey1 = l1 * sinf(theta1);
    const float ex2 = half_base + l2 * cosf(theta2);
    const float ey2 = l2 * sinf(theta2);
    const float dx = ex2 - ex1;
    const float dy = ey2 - ey1;
    const float d = sqrtf(dx * dx + dy * dy);

    if (d < APP_EPSILON_F || d > (l3 + l4) || d < fabsf(l3 - l4)) {
        return false;
    }

    const float a = (l3 * l3 - l4 * l4 + d * d) / (2.0f * d);
    const float h2 = l3 * l3 - a * a;
    if (h2 < 0.0f) {
        return false;
    }

    const float h = sqrtf(h2);
    const float mx = ex1 + a * dx / d;
    const float my = ey1 + a * dy / d;
    const float rx = -dy / d;
    const float ry = dx / d;
    const float ix1 = mx + h * rx;
    const float iy1 = my + h * ry;
    const float ix2 = mx - h * rx;
    const float iy2 = my - h * ry;

    if (iy1 >= iy2) {
        pose->x_um = (int32_t)lroundf(ix1);
        pose->y_um = (int32_t)lroundf(iy1);
    } else {
        pose->x_um = (int32_t)lroundf(ix2);
        pose->y_um = (int32_t)lroundf(iy2);
    }
    return true;
}

bool ScaraKinematics_JointToPulse(int32_t theta1_mrad, int32_t theta2_mrad,
                                  int64_t *pulse1, int64_t *pulse2)
{
    if (pulse1 == NULL || pulse2 == NULL) {
        return false;
    }
    if (!joint_in_range(theta1_mrad, theta2_mrad)) {
        return false;
    }

    const AppParams *p = AppParams_Get();
    const float j1 = ((float)theta1_mrad - (float)p->motor_zero_mrad[0]) / 1000.0f;
    const float j2 = ((float)theta2_mrad - (float)p->motor_zero_mrad[1]) / 1000.0f;
    const float scale1 = (float)p->pulses_per_rev[0] * (float)p->reducer_ratio[0];
    const float scale2 = (float)p->pulses_per_rev[1] * (float)p->reducer_ratio[1];
    *pulse1 = (int64_t)llroundf(j1 * (float)p->motor_dir_sign[0] * scale1 / APP_RAD_PER_REV_F);
    *pulse2 = (int64_t)llroundf(j2 * (float)p->motor_dir_sign[1] * scale2 / APP_RAD_PER_REV_F);
    return true;
}

bool ScaraKinematics_PulseToJoint(int64_t pulse1, int64_t pulse2, ScaraJoint *joint)
{
    if (joint == NULL) {
        return false;
    }

    const AppParams *p = AppParams_Get();
    const float scale1 = (float)p->pulses_per_rev[0] * (float)p->reducer_ratio[0];
    const float scale2 = (float)p->pulses_per_rev[1] * (float)p->reducer_ratio[1];
    if (scale1 <= 0.0f || scale2 <= 0.0f) {
        return false;
    }
    joint->theta1_mrad = (int32_t)lroundf(
        (float)pulse1 * (float)p->motor_dir_sign[0] * APP_RAD_PER_REV_F * 1000.0f / scale1
    ) + p->motor_zero_mrad[0];
    joint->theta2_mrad = (int32_t)lroundf(
        (float)pulse2 * (float)p->motor_dir_sign[1] * APP_RAD_PER_REV_F * 1000.0f / scale2
    ) + p->motor_zero_mrad[1];
    return joint_in_range(joint->theta1_mrad, joint->theta2_mrad);
}

bool ScaraKinematics_InverseUm(int32_t x_um, int32_t y_um, ScaraJoint *joint)
{
    /* XY 到双主动关节角；若目标超出机构几何范围，返回 false。 */
    if (joint == NULL) {
        return false;
    }

    float theta1 = 0.0f;
    float theta2 = 0.0f;
    if (!inverse_um_radians(x_um, y_um, &theta1, &theta2)) {
        return false;
    }
    joint->theta1_mrad = (int32_t)lroundf(theta1 * 1000.0f);
    joint->theta2_mrad = (int32_t)lroundf(theta2 * 1000.0f);
    return true;
}

bool ScaraKinematics_Forward(const ScaraJoint *joint, ScaraPose *pose)
{
    /* 双主动关节角到末端 XY，用于 STATUS/UI 显示软件估算位置。 */
    if (joint == NULL || pose == NULL || !joint_in_range(joint->theta1_mrad, joint->theta2_mrad)) {
        return false;
    }

    const float t1 = (float)joint->theta1_mrad / 1000.0f;
    const float t2 = (float)joint->theta2_mrad / 1000.0f;
    return forward_radians(t1, t2, pose);
}

bool ScaraKinematics_InverseUmToPulse(int32_t x_um, int32_t y_um,
                                      int64_t *pulse1, int64_t *pulse2)
{
    if (pulse1 == NULL || pulse2 == NULL) {
        return false;
    }

    float theta1 = 0.0f;
    float theta2 = 0.0f;
    if (!inverse_um_radians(x_um, y_um, &theta1, &theta2)) {
        return false;
    }

    const AppParams *p = AppParams_Get();
    const float zero1 = (float)p->motor_zero_mrad[0] / 1000.0f;
    const float zero2 = (float)p->motor_zero_mrad[1] / 1000.0f;
    const float scale1 = (float)p->pulses_per_rev[0] * (float)p->reducer_ratio[0];
    const float scale2 = (float)p->pulses_per_rev[1] * (float)p->reducer_ratio[1];
    if (scale1 <= 0.0f || scale2 <= 0.0f) {
        return false;
    }
    *pulse1 = (int64_t)llroundf(
        (theta1 - zero1) * (float)p->motor_dir_sign[0] * scale1 / APP_RAD_PER_REV_F
    );
    *pulse2 = (int64_t)llroundf(
        (theta2 - zero2) * (float)p->motor_dir_sign[1] * scale2 / APP_RAD_PER_REV_F
    );
    return true;
}

bool ScaraKinematics_PulseToPose(int64_t pulse1, int64_t pulse2, ScaraPose *pose)
{
    if (pose == NULL) {
        return false;
    }

    const AppParams *p = AppParams_Get();
    const float scale1 = (float)p->pulses_per_rev[0] * (float)p->reducer_ratio[0];
    const float scale2 = (float)p->pulses_per_rev[1] * (float)p->reducer_ratio[1];
    if (scale1 <= 0.0f || scale2 <= 0.0f) {
        return false;
    }

    const float theta1 = (float)pulse1 * (float)p->motor_dir_sign[0] * APP_RAD_PER_REV_F / scale1 +
                         (float)p->motor_zero_mrad[0] / 1000.0f;
    const float theta2 = (float)pulse2 * (float)p->motor_dir_sign[1] * APP_RAD_PER_REV_F / scale2 +
                         (float)p->motor_zero_mrad[1] / 1000.0f;
    if (!joint_in_range((int32_t)lroundf(theta1 * 1000.0f),
                        (int32_t)lroundf(theta2 * 1000.0f))) {
        return false;
    }
    return forward_radians(theta1, theta2, pose);
}
