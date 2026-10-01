#include "direction_mapping.h"

/* 板面标定轴: 板面角 = 180° - SRP原始角.
 * SRP 的 0° 沿麦阵 X+ 轴 (mic2 方向), 与板面丝印 0° 差 180°;
 * 若整机换壳后所有方向统一偏一个固定角, 改这里补偿. */
#define BOARD_DOA_MIRROR_AXIS_DEG (180.0f)

static const float s_led_angles_deg[DIRECTION_LED_COUNT] = {
    /* 16 灯均布 22.5° 间隔. 电气序: LED11(pixel0)->LED17(pixel15),
     * 从 GPIO1 链头起顺时针步进 -22.5° (标定后若反向, 把符号翻转即可):
     * pixel0=LED11 在 0°, 之后每颗逆时针 22.5° */
    0.0f, 337.5f, 315.0f, 292.5f, 270.0f, 247.5f, 225.0f, 202.5f,
    180.0f, 157.5f, 135.0f, 112.5f, 90.0f, 67.5f, 45.0f, 22.5f,
};

static float normalize_angle(float angle_deg)
{
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

float direction_calibrate_doa(float raw_angle_deg)
{
    return normalize_angle(BOARD_DOA_MIRROR_AXIS_DEG - raw_angle_deg);
}

int direction_led_nearest_index(float angle_deg)
{
    const float angle = normalize_angle(angle_deg);
    int nearest_index = 0;
    float nearest_distance = 360.0f;

    for (int i = 0; i < DIRECTION_LED_COUNT; ++i) {
        float distance = angle - s_led_angles_deg[i];
        if (distance < 0.0f) {
            distance = -distance;
        }
        if (distance > 180.0f) {
            distance = 360.0f - distance;
        }
        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest_index = i;
        }
    }

    return nearest_index;
}

float direction_led_angle_of_index(int index)
{
    if (index < 0 || index >= DIRECTION_LED_COUNT) {
        return 0.0f;
    }
    return s_led_angles_deg[index];
}
