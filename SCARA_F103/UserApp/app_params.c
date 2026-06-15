#include "app_params.h"

/* 运行参数管理：默认参数来自 app_config.h，可保存到最后 1KB Flash 参数页。 */

#include "app_config.h"
#include "stm32f1xx_hal.h"

#include <string.h>

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t crc;
    AppParams params;
} AppParamsFlashRecord;

static AppParams s_params;

static uint32_t params_crc(const AppParams *params)
{
    const uint8_t *bytes = (const uint8_t *)params;
    uint32_t crc = 0x811C9DC5u;

    for (uint32_t i = 0; i < sizeof(AppParams); ++i) {
        crc ^= bytes[i];
        crc *= 16777619u;
    }
    return crc;
}

int32_t AppParams_NormalizeSign(int32_t sign)
{
    return sign < 0 ? -1 : 1;
}

void AppParams_Defaults(void)
{
    s_params.pulses_per_rev[0] = APP_PULSES_PER_REV_M1;
    s_params.pulses_per_rev[1] = APP_PULSES_PER_REV_M2;
    s_params.reducer_ratio[0] = APP_REDUCER_RATIO_M1;
    s_params.reducer_ratio[1] = APP_REDUCER_RATIO_M2;
    s_params.motor_dir_sign[0] = AppParams_NormalizeSign(APP_MOTOR1_DIR_SIGN);
    s_params.motor_dir_sign[1] = AppParams_NormalizeSign(APP_MOTOR2_DIR_SIGN);
    s_params.motor_zero_mrad[0] = APP_MOTOR1_ZERO_MRAD;
    s_params.motor_zero_mrad[1] = APP_MOTOR2_ZERO_MRAD;
    s_params.motor_min_pos = APP_MOTOR_MIN_POS;
    s_params.motor_max_pos = APP_MOTOR_MAX_POS;
    s_params.scara_base_um = APP_SCARA_BASE_UM;
    s_params.active_arm_um[0] = APP_SCARA_ACTIVE1_UM;
    s_params.active_arm_um[1] = APP_SCARA_ACTIVE2_UM;
    s_params.passive_arm_um[0] = APP_SCARA_PASSIVE1_UM;
    s_params.passive_arm_um[1] = APP_SCARA_PASSIVE2_UM;
    s_params.theta_min_mrad[0] = APP_SCARA_THETA1_MIN_MRAD;
    s_params.theta_min_mrad[1] = APP_SCARA_THETA2_MIN_MRAD;
    s_params.theta_max_mrad[0] = APP_SCARA_THETA1_MAX_MRAD;
    s_params.theta_max_mrad[1] = APP_SCARA_THETA2_MAX_MRAD;
    s_params.ik_left_elbow_sign = AppParams_NormalizeSign(APP_SCARA_IK_LEFT_ELBOW_SIGN);
    s_params.ik_right_elbow_sign = AppParams_NormalizeSign(APP_SCARA_IK_RIGHT_ELBOW_SIGN);
    s_params.movl_segment_um = APP_MOVL_SEGMENT_UM;
    s_params.backlash_pulses[0] = APP_BACKLASH_PULSES_M1;
    s_params.backlash_pulses[1] = APP_BACKLASH_PULSES_M2;
}

void AppParams_Init(void)
{
    AppParams_Defaults();
    (void)AppParams_Load();
}

const AppParams *AppParams_Get(void)
{
    return &s_params;
}

AppParams *AppParams_Mutable(void)
{
    return &s_params;
}

bool AppParams_Load(void)
{
    const AppParamsFlashRecord *record = (const AppParamsFlashRecord *)APP_PARAM_FLASH_ADDR;

    if (record->magic != APP_PARAM_FLASH_MAGIC ||
        record->version != APP_PARAM_FLASH_VERSION ||
        record->size != sizeof(AppParams) ||
        record->crc != params_crc(&record->params)) {
        return false;
    }

    memcpy(&s_params, &record->params, sizeof(s_params));
    s_params.motor_dir_sign[0] = AppParams_NormalizeSign(s_params.motor_dir_sign[0]);
    s_params.motor_dir_sign[1] = AppParams_NormalizeSign(s_params.motor_dir_sign[1]);
    s_params.ik_left_elbow_sign = AppParams_NormalizeSign(s_params.ik_left_elbow_sign);
    s_params.ik_right_elbow_sign = AppParams_NormalizeSign(s_params.ik_right_elbow_sign);
    return true;
}

bool AppParams_Save(void)
{
    AppParamsFlashRecord record;
    memset(&record, 0xFF, sizeof(record));
    record.magic = APP_PARAM_FLASH_MAGIC;
    record.version = APP_PARAM_FLASH_VERSION;
    record.size = sizeof(AppParams);
    record.params = s_params;
    record.crc = params_crc(&record.params);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = APP_PARAM_FLASH_ADDR;
    erase.NbPages = 1;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    const uint16_t *src = (const uint16_t *)&record;
    uint32_t address = APP_PARAM_FLASH_ADDR;
    uint32_t halfwords = (sizeof(record) + 1u) / 2u;
    for (uint32_t i = 0; i < halfwords; ++i) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        address += 2u;
    }

    HAL_FLASH_Lock();
    return AppParams_Load();
}
