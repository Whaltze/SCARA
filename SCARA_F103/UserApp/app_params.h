#ifndef APP_PARAMS_H
#define APP_PARAMS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t pulses_per_rev[2];
    int32_t reducer_ratio[2];
    int32_t motor_dir_sign[2];
    int32_t motor_zero_mrad[2];
    int32_t motor_min_pos;
    int32_t motor_max_pos;
    int32_t scara_base_um;
    int32_t active_arm_um[2];
    int32_t passive_arm_um[2];
    int32_t theta_min_mrad[2];
    int32_t theta_max_mrad[2];
    int32_t ik_left_elbow_sign;
    int32_t ik_right_elbow_sign;
    int32_t movl_segment_um;
    /* 每电机反向间隙补偿脉冲（换向时多发脉冲吃旷量；0=不补偿）。 */
    int32_t backlash_pulses[2];
} AppParams;

void AppParams_Init(void);
void AppParams_Defaults(void);
const AppParams *AppParams_Get(void);
AppParams *AppParams_Mutable(void);
bool AppParams_Load(void);
bool AppParams_Save(void);
int32_t AppParams_NormalizeSign(int32_t sign);

#endif
