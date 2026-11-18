#include "doa_led_scene.h"
#include "direction_mapping.h"
#include <math.h>
#include <string.h>

/* 多方向额外保守门槛。单方向首次更新无需这些等待。时间单位均为毫秒，
 * 不按帧猜时间；算法处理偶尔延迟也不会缩短确认时长。 */
#define MULTI_CONFIRM_MS       500U  /* 仅灯效：双向共同稳定半秒，防短暂多灯闪现 */
#define MULTI_CONFIRM_FRAMES   12U   /* 同时要求十二次观测，不让少数瞬间峰变成第二灯 */
#define MULTI_MATCH_DEG        18.0f /* 相邻观测允许的角波动 */
#define MULTI_BRIDGE_MS        120U  /* 短暂缺峰保持；超过即回到可靠的主方向 */

static float distance(float a, float b)
{
    float d = fabsf(a - b);
    return d > 180 ? 360 - d : d;
}

void doa_led_scene_init(doa_led_scene_t *state) { memset(state, 0, sizeof(*state)); }

void doa_led_scene_update(doa_led_scene_t *state, const gcc_phat_result_t *spatial,
                           bool pair_usable, bool primary_visible, float primary_angle,
                           float band_rms, float noise_rms, uint32_t now_ms,
                           led_effect_scene_t *scene)
{
    if (!state || !scene) return;
    memset(scene, 0, sizeof(*scene));
    bool pair = pair_usable && spatial && spatial->display_peak_count == 2;
    float angles[2] = {0}, levels[2] = {0};
    if (pair) {
        for (int i = 0; i < 2; ++i) {
            const gcc_phat_peak_t *peak = &spatial->display_peaks[i];
            if (!isfinite(peak->angle_deg) || !isfinite(peak->directional_rms) ||
                !isfinite(noise_rms) || noise_rms < 0 || peak->directional_rms <= noise_rms) {
                pair = false;
                break;
            }
            angles[i] = direction_calibrate_doa(peak->angle_deg);
            levels[i] = led_effect_level_from_rms(peak->directional_rms, noise_rms);
        }
        if (distance(angles[0], angles[1]) < 60.0f) pair = false;
    }
    if (pair) {
        /* 峰序可能随声强变化：选择与原方向距离总和更小的关联。 */
        if (state->candidate && distance(angles[0], state->angles[1]) + distance(angles[1], state->angles[0]) <
                                distance(angles[0], state->angles[0]) + distance(angles[1], state->angles[1])) {
            float tmp = angles[0]; angles[0] = angles[1]; angles[1] = tmp;
            tmp = levels[0]; levels[0] = levels[1]; levels[1] = tmp;
        }
        bool matches = state->candidate &&
                       (uint32_t)(now_ms - state->last_pair_ms) <= MULTI_BRIDGE_MS &&
                       distance(angles[0], state->angles[0]) <= MULTI_MATCH_DEG &&
                       distance(angles[1], state->angles[1]) <= MULTI_MATCH_DEG;
        if (!matches) {
            state->candidate_since_ms = now_ms;
            state->candidate_frames = 0;
            state->confirmed = false;
        }
        for (int i = 0; i < 2; ++i) { state->angles[i] = angles[i]; state->levels[i] = levels[i]; }
        state->candidate = true;
        state->last_pair_ms = now_ms;
        if (state->candidate_frames < MULTI_CONFIRM_FRAMES) state->candidate_frames++;
        if (state->candidate_frames >= MULTI_CONFIRM_FRAMES &&
            (uint32_t)(now_ms - state->candidate_since_ms) >= MULTI_CONFIRM_MS) state->confirmed = true;
    } else if (!state->confirmed || !pair_usable ||
               (uint32_t)(now_ms - state->last_pair_ms) > MULTI_BRIDGE_MS) {
        /* 未确认时必须连续：不能把间歇闪现的假峰跨空帧累计成第二声源。
         * 只有已经稳定显示的双源才允许短暂缺峰保持。 */
        state->candidate = false;
        state->confirmed = false;
        state->candidate_frames = 0;
    }

    if (state->confirmed) {
        scene->count = 2;
        for (int i = 0; i < 2; ++i) {
            scene->sources[i] = (led_effect_source_t){state->angles[i], state->levels[i]};
        }
    } else if (primary_visible && isfinite(primary_angle)) {
        scene->count = 1;
        scene->sources[0] = (led_effect_source_t){primary_angle,
                                                 led_effect_level_from_rms(band_rms, noise_rms)};
    }
}
