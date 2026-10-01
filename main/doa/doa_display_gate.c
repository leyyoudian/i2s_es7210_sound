#include "doa_display_gate.h"

#include "direction_mapping.h"

#include <math.h>
#include <string.h>

/* 帧率 46.9fps (1024 点 @48kHz), 帧数为原 16kHz 配置的 3 倍以保持相同时间常数 */
#define STALE_WEAK_REJECT_FRAMES 15U  /* ~320ms */
#define WEAK_REJECT_ENERGY       (0.0015f)
#define CANDIDATE_CONFIRM_FRAMES 2U   /* ~43ms */
/* 灯位切换滞回: 距当前灯位中心超过该角度才允许开始候选切换.
 * 半灯位 11.25° < 滞回 < 全灯位 22.5°, 使估计噪声在分界附近
 * 无法反复触发相邻灯切换 (离开需 >14°, 回来需 <8.5°). */
#define SWITCH_HYSTERESIS_DEG    (14.0f)
/* 显示角度死区: 灯位未变时, 角度波动小于该值不更新 display_angle,
 * 网页罗盘的数字/指针不再跟随估计噪声逐帧微抖. */
#define DISPLAY_DEAD_ZONE_DEG    (5.0f)

static float angle_distance_deg(float a, float b)
{
    float d = fabsf(a - b);
    return d > 180.0f ? 360.0f - d : d;
}

static float gated_display_angle(float live_deg, float current_deg)
{
    return angle_distance_deg(live_deg, current_deg) >= DISPLAY_DEAD_ZONE_DEG
        ? live_deg : current_deg;
}

void doa_display_gate_init(doa_display_gate_t *gate)
{
    if (!gate) return;
    memset(gate, 0, sizeof(*gate));
    gate->held_pixel = -1;
    gate->candidate_pixel = -1;
}

bool doa_display_gate_update(doa_display_gate_t *gate, bool tracker_updated,
                             bool source_active, bool localization_valid,
                             float doa_energy, float angle_deg,
                             float *display_angle_deg)
{
    if (!gate || !display_angle_deg) return false;

    if (tracker_updated && source_active && localization_valid &&
        isfinite(angle_deg)) {
        bool force = !gate->display_active || gate->held_pixel < 0;
        int pixel;
        if (!force &&
            angle_distance_deg(angle_deg,
                               direction_led_angle_of_index(gate->held_pixel)) <
                SWITCH_HYSTERESIS_DEG) {
            /* 滞回死区内: 仍视为原灯位, 不启动候选切换 */
            pixel = gate->held_pixel;
        } else {
            pixel = direction_led_nearest_index(angle_deg);
        }
        gate->stale_weak_reject_frames = 0;
        if (force || pixel == gate->held_pixel) {
            gate->held_pixel = pixel;
            gate->candidate_pixel = -1;
            gate->candidate_count = 0;
            gate->display_angle = force
                ? angle_deg
                : gated_display_angle(angle_deg, gate->display_angle);
            gate->display_active = true;
            *display_angle_deg = gate->display_angle;
            return true;
        }

        if (pixel != gate->candidate_pixel) {
            gate->candidate_pixel = pixel;
            gate->candidate_count = 1;
        } else if (gate->candidate_count < CANDIDATE_CONFIRM_FRAMES) {
            gate->candidate_count++;
        }

        if (gate->candidate_count >= CANDIDATE_CONFIRM_FRAMES) {
            gate->held_pixel = pixel;
            gate->candidate_pixel = -1;
            gate->candidate_count = 0;
            gate->display_angle = angle_deg;
        }

        *display_angle_deg = gate->display_angle;
        return gate->display_active;
    }

    if (!source_active) {
        gate->stale_weak_reject_frames = 0;
        gate->candidate_pixel = -1;
        gate->candidate_count = 0;
        gate->display_active = false;
        *display_angle_deg = gate->display_angle;
        return false;
    }

    bool weak_rejected = !localization_valid &&
        (!isfinite(doa_energy) || doa_energy < WEAK_REJECT_ENERGY);
    if (weak_rejected) {
        if (++gate->stale_weak_reject_frames >= STALE_WEAK_REJECT_FRAMES) {
            gate->display_active = false;
            gate->candidate_pixel = -1;
            gate->candidate_count = 0;
        }
    } else {
        gate->stale_weak_reject_frames = 0;
    }

    *display_angle_deg = gate->display_angle;
    return gate->display_active;
}
