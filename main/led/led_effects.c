#include "led_effects.h"
#include <math.h>
#include <string.h>

static float clamp01(float x) { return fmaxf(0.0f, fminf(1.0f, x)); }
static float wrap(float x) { x = fmodf(x, 360.0f); return x < 0 ? x + 360.0f : x; }
static float delta(float a, float b)
{
    float d = wrap(a - b + 180.0f) - 180.0f;
    return d;
}

void led_effects_init(led_effects_t *state) { memset(state, 0, sizeof(*state)); }

float led_effect_level_from_rms(float signal_rms, float noise_rms)
{
    if (!isfinite(signal_rms) || !isfinite(noise_rms) || signal_rms <= 0 || noise_rms < 0) return 0;
    /* RMS 能量相减后再开方；将背景噪声与有用声音的能量区别开。 */
    float power = signal_rms * signal_rms - noise_rms * noise_rms;
    if (power <= 1e-12f) return 0;
    float db = 10.0f * log10f(power);
    return clamp01((db - LED_EFFECT_QUIET_DBFS) /
                   (LED_EFFECT_LOUD_DBFS - LED_EFFECT_QUIET_DBFS));
}

static void color(float level, float *r, float *g, float *b)
{
#if LED_EFFECT_FIXED_COLOR
    (void)level;
    *r = LED_EFFECT_FIXED_RED / 255.0f;
    *g = LED_EFFECT_FIXED_GREEN / 255.0f;
    *b = LED_EFFECT_FIXED_BLUE / 255.0f;
#else
    if (level < .5f) { *r = 0; *g = level * 2; *b = 1.0f - level; }
    else { *r = (level - .5f) * 2; *g = 1; *b = (1 - level) * .5f; }
#endif
}

static void update_visual_target(led_effect_track_t *track, float input, uint32_t now_ms)
{
    /* 连续小步运动与孤立A-B-A抖动分开处理。不再要求每一颗灯停450ms，
     * 否则转圈时目标一直在变，候选永远不能确认，显示就会卡住或跳格。 */
    float step = delta(input, track->last_input_angle);
    track->last_input_angle = input;
    if (fabsf(step) >= LED_EFFECT_MOTION_MIN_STEP) {
        int sign = step > 0 ? 1 : -1;
        if (fabsf(step) > LED_EFFECT_MOTION_MAX_STEP) track->motion_steps = 0;
        else {
            if (sign != track->input_motion_sign ||
                (uint32_t)(now_ms - track->last_motion_ms) > LED_EFFECT_MOTION_HOLD_MS)
                track->motion_steps = 0;
            if (track->motion_steps < LED_EFFECT_MOTION_STEPS) track->motion_steps++;
            track->input_motion_sign = sign;
            track->last_motion_ms = now_ms;
        }
    }
    float movement = fabsf(delta(input, track->accepted_angle_deg));
    if (movement <= LED_EFFECT_ANGLE_DEAD_ZONE) { track->pending = false; return; }
    bool moving = track->motion_steps >= LED_EFFECT_MOTION_STEPS &&
                  (uint32_t)(now_ms - track->last_motion_ms) <= LED_EFFECT_MOTION_HOLD_MS;
    if (moving && movement < LED_EFFECT_JUMP_DEG) {
        track->accepted_angle_deg = input;
        track->pending = false;
        return;
    }
    if (!track->pending || fabsf(delta(input, track->pending_angle_deg)) > LED_EFFECT_PENDING_TOLERANCE_DEG) {
        track->pending = true;
        track->pending_angle_deg = input;
        track->pending_since_ms = now_ms;
    }
    uint32_t confirm = movement >= LED_EFFECT_JUMP_DEG
        ? LED_EFFECT_LARGE_CONFIRM_MS : LED_EFFECT_STEP_CONFIRM_MS;
    if ((uint32_t)(now_ms - track->pending_since_ms) >= confirm) {
        track->accepted_angle_deg = track->pending_angle_deg;
        track->pending = false;
        if (movement >= LED_EFFECT_JUMP_DEG) {
            track->angle_deg = track->accepted_angle_deg;
            track->head_pixel = direction_led_nearest_index(track->angle_deg);
            track->trail = false;
            track->motion_steps = 0;
        }
    }
}

void led_effects_render(led_effects_t *state, const led_effect_scene_t *scene,
                         uint32_t now_ms, led_effect_rgb_t pixels[DIRECTION_LED_COUNT])
{
    if (!state || !pixels) return;
    float accum[DIRECTION_LED_COUNT][3] = {{0}};
    unsigned n = scene ? scene->count : 0;
    if (n > LED_EFFECT_MAX_SOURCES) n = LED_EFFECT_MAX_SOURCES;
    float dt = state->initialized ? (float)(uint32_t)(now_ms - state->last_ms) : LED_EFFECT_FRAME_MS;
    dt = fminf(100.0f, dt);
    state->last_ms = now_ms;
    state->initialized = true;
    bool used[LED_EFFECT_MAX_SOURCES] = {false};
    unsigned order[LED_EFFECT_MAX_SOURCES];
    for (unsigned i = 0; i < LED_EFFECT_MAX_SOURCES; ++i) order[i] = i;
    if (n == 1 && state->tracks[1].initialized && isfinite(scene->sources[0].angle_deg) &&
        (!state->tracks[0].initialized ||
         fabsf(delta(scene->sources[0].angle_deg, state->tracks[1].angle_deg)) <
         fabsf(delta(scene->sources[0].angle_deg, state->tracks[0].angle_deg)))) {
        /* 只剩第二声源时先关联它，不能让第一轨抢走输入、造成重复增亮。 */
        order[0] = 1; order[1] = 0;
    }
    float breath = 1.0f;
#if LED_EFFECT_MODE == LED_EFFECT_MODE_BREATHE
    float phase = (float)(now_ms % LED_EFFECT_BREATH_PERIOD_MS) / LED_EFFECT_BREATH_PERIOD_MS;
    breath = LED_EFFECT_BREATH_MIN + (1 - LED_EFFECT_BREATH_MIN) *
             .5f * (1 + cosf(6.28318530718f * phase));
#elif LED_EFFECT_MODE == LED_EFFECT_MODE_BLINK
    breath = now_ms % LED_EFFECT_BLINK_PERIOD_MS < LED_EFFECT_BLINK_PERIOD_MS / 2 ? 1.0f : 0.0f;
#endif
    /* 方向按角距离关联，不按峰值排序关联，防止两个音源强弱交换时灯绕圈跳。 */
    for (unsigned slot = 0; slot < LED_EFFECT_MAX_SOURCES; ++slot) {
        unsigned i = order[slot];
        led_effect_track_t *track = &state->tracks[i];
        int match = -1;
        float best = 361;
        if (track->initialized) {
            for (unsigned j = 0; j < n; ++j) {
                if (used[j] || !isfinite(scene->sources[j].angle_deg) ||
                    !isfinite(scene->sources[j].level)) continue;
                float d = fabsf(delta(scene->sources[j].angle_deg, track->angle_deg));
                if (d < best) { best = d; match = (int)j; }
            }
        } else {
            for (unsigned j = 0; j < n; ++j) {
                if (!used[j] && isfinite(scene->sources[j].angle_deg) &&
                    isfinite(scene->sources[j].level)) { match = (int)j; break; }
            }
        }
        if (match >= 0) {
            used[match] = true;
            float angle = wrap(scene->sources[match].angle_deg);
            if (!track->initialized) {
                track->angle_deg = angle;
                track->accepted_angle_deg = angle;
                track->last_input_angle = angle;
                track->head_pixel = direction_led_nearest_index(angle);
                track->motion_steps = 0;
                track->pending = false;
                track->trail = false;
                track->level = clamp01(scene->sources[match].level);
                track->brightness = LED_EFFECT_BRIGHTNESS_MIN +
                    (LED_EFFECT_BRIGHTNESS_MAX - LED_EFFECT_BRIGHTNESS_MIN) * track->level;
                track->initialized = true;
            } else {
                update_visual_target(track, angle, now_ms);
                float movement = delta(track->accepted_angle_deg, track->angle_deg) *
                                 dt / (LED_EFFECT_FOLLOW_MS + dt);
                track->angle_deg = wrap(track->angle_deg + movement);
                if (fabsf(delta(track->accepted_angle_deg, track->angle_deg)) < .1f)
                    track->angle_deg = track->accepted_angle_deg;
                if (fabsf(movement) >= .15f) {
                    track->motion_sign = movement > 0 ? 1 : -1;
                    track->trail_since_ms = now_ms;
                    track->trail = n == 1;
                }
            }
            track->target_level = clamp01(scene->sources[match].level);
        } else {
            track->target_level = -1; /* -1 表示失活；0 是最弱的有效声源。 */
            track->pending = false;
            /* 从双源回到单源时立即撤下未匹配的旧方向，不能让旧轨淡出与新轨
             * 同时亮而伪装成第三/第二声源。全部无源时仍保留整体柔和淡出。 */
            if (n == 1) { track->initialized = false; track->trail = false; continue; }
        }

        if (!track->initialized) continue;
        float target = track->target_level >= 0
            ? LED_EFFECT_BRIGHTNESS_MIN + (LED_EFFECT_BRIGHTNESS_MAX - LED_EFFECT_BRIGHTNESS_MIN) * track->target_level : 0;
        float current = track->brightness;
        float tau = target > current ? LED_EFFECT_ATTACK_MS : LED_EFFECT_RELEASE_MS;
        current += (target - current) * dt / (tau + dt);
        track->brightness = current;
        if (track->target_level >= 0) track->level +=
            (track->target_level - track->level) * dt / (tau + dt);
        if (target == 0 && current < .6f) { track->initialized = false; continue; }
        float r, g, b;
        color(clamp01(track->level), &r, &g, &b);
        bool moving = track->trail && (uint32_t)(now_ms - track->trail_since_ms) < LED_EFFECT_TRAIL_MS;
        float brightness = current * (moving ? 1.0f : breath); /* 移动时主灯稳亮，不边走边呼吸闪 */
        /* 主灯跟随已稳定的显示目标。单源只允许一颗相连尾灯，不能把多个
         * 历史方向叠加。双源时只画各自主灯，避免两条尾巴造成四灯闪现。 */
        float from_center = fabsf(delta(track->angle_deg,
                                        direction_led_angle_of_index(track->head_pixel)));
        if (from_center > 11.25f + LED_EFFECT_HEAD_HYST_DEG)
            track->head_pixel = direction_led_nearest_index(track->angle_deg);
        int first = track->head_pixel;
        accum[first][0] += r * brightness;
        accum[first][1] += g * brightness;
        accum[first][2] += b * brightness;
        if (track->trail && n <= 1) {
            uint32_t age = now_ms - track->trail_since_ms;
            if (age >= LED_EFFECT_TRAIL_MS || track->motion_sign == 0) track->trail = false;
            else {
                float fade = 1.0f - (float)age / LED_EFFECT_TRAIL_MS;
                float tail = brightness * LED_EFFECT_TRAIL_RATIO * fade * fade;
                int tail_pixel = direction_led_nearest_index(wrap(direction_led_angle_of_index(first) -
                                                                  track->motion_sign * 22.5f));
                accum[tail_pixel][0] += r * tail;
                accum[tail_pixel][1] += g * tail;
                accum[tail_pixel][2] += b * tail;
            }
        } else track->trail = false;
    }
    for (int i = 0; i < DIRECTION_LED_COUNT; ++i) {
        pixels[i].r = (uint8_t)lrintf(fminf(LED_EFFECT_BRIGHTNESS_MAX, accum[i][0]));
        pixels[i].g = (uint8_t)lrintf(fminf(LED_EFFECT_BRIGHTNESS_MAX, accum[i][1]));
        pixels[i].b = (uint8_t)lrintf(fminf(LED_EFFECT_BRIGHTNESS_MAX, accum[i][2]));
    }
}
