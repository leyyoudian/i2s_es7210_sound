#pragma once
#include "gcc_phat.h"
#include "led_effects.h"

/* 主方向仍由已有 tracker + display_gate 决定。该层只对双峰持续确认，
 * 防止一个方向的回声或排序交换直接变成“多个声源”。不声称能区分全部反射。 */
typedef struct {
    float angles[2];
    float levels[2];
    uint32_t candidate_since_ms;
    uint32_t last_pair_ms;
    unsigned candidate_frames;
    bool confirmed;
    bool candidate;
} doa_led_scene_t;

void doa_led_scene_init(doa_led_scene_t *state);
void doa_led_scene_update(doa_led_scene_t *state, const gcc_phat_result_t *spatial,
                           bool pair_usable, bool primary_visible, float primary_angle,
                           float band_rms, float noise_rms, uint32_t now_ms,
                           led_effect_scene_t *scene);
