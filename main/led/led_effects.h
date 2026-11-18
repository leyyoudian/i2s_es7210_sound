#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "direction_mapping.h"
#include "led_effects_config.h"

/* scene 只包含已由定位/显示层认可的声源，角度为板面坐标 0~360°。
 * level 为 0~1 的噪声扣除后强度；不是 PHAT 置信度，也不是米数。 */
typedef struct { float angle_deg; float level; } led_effect_source_t;
typedef struct {
    unsigned count;
    led_effect_source_t sources[LED_EFFECT_MAX_SOURCES];
} led_effect_scene_t;
typedef struct { uint8_t r, g, b; } led_effect_rgb_t;
typedef struct {
    float angle_deg, level, brightness, target_level;
    float accepted_angle_deg, pending_angle_deg;
    float last_input_angle;
    uint32_t pending_since_ms, last_motion_ms;
    uint32_t trail_since_ms;
    int head_pixel, motion_sign, input_motion_sign;
    unsigned motion_steps;
    bool pending;
    bool trail;
    bool initialized;
} led_effect_track_t;
typedef struct {
    led_effect_track_t tracks[LED_EFFECT_MAX_SOURCES];
    uint32_t last_ms;
    bool initialized;
} led_effects_t;

void led_effects_init(led_effects_t *state);
float led_effect_level_from_rms(float signal_rms, float noise_rms);
/* 纯函数式状态推进：用真实毫秒驱动呼吸和淡入淡出，主机可指定时间验证。
 * 每帧先合成所有方向，再由硬件层一次刷新，绝不“清灯→点灯”分两次发。 */
void led_effects_render(led_effects_t *state, const led_effect_scene_t *scene,
                         uint32_t now_ms, led_effect_rgb_t pixels[DIRECTION_LED_COUNT]);
