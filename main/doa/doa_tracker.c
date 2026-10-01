#include "doa_tracker.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define REQUIRED_STABLE_FRAMES 2U
#define LARGE_JUMP_DEG 60.0f
#define PENDING_SPREAD_DEG 20.0f
#define HIGH_CONFIDENCE_JUMP 0.35f
#define HIGH_CONFIDENCE_JUMP_FRAMES 2U
#define TRANSIENT_GUARD_FRAMES 6U
/* 瞬态(响指)路径的对侧镜像保护: 瞬态只有 1~2 帧快照, 无法用连续帧区分镜像,
 * 故对侧跳变需更高的绝对置信才接受, 否则保持原方向(避免响指亮对面). */
#define TRANSIENT_OPPOSITE_JUMP_DEG 120.0f
#define TRANSIENT_OPPOSITE_MIN_CONF 0.60f
/* 瞬态冷启动锁定: 跟踪器尚未初始化时, 镜像伪影没有任何历史方向可对照,
 * 必须要求较高置信才允许瞬态直接锁定初始方向, 否则回落到普通路径的
 * 连续帧确认. */
#define TRANSIENT_INITIAL_LOCK_MIN_CONF 0.35f
#define MAX_PENDING_MISSES 3U
/* 对侧(镜像)跳变是 GCC-PHAT 对称伪影, 最容易假触发.
 * 区分镜像假峰 vs 真实声源切换靠"连续稳定帧数": 镜像偶发 1~2 帧即消失,
 * 真实切换持续多帧. 因此用中等置信 + 连续 5 帧稳定, 而非过高的绝对置信,
 * 否则真实 0°->180° 切换会被锁死成跑马灯. */
#define OPPOSITE_JUMP_DEG 120.0f
#define OPPOSITE_STABLE_FRAMES 5U
#define OPPOSITE_MIN_CONFIDENCE 0.18f
#define MEDIUM_JUMP_DEG 60.0f
#define WIDE_JUMP_DEG 90.0f
#define MEDIUM_CONFIDENCE 0.25f
#define WIDE_CONFIDENCE 0.35f

static float circular_distance(float a, float b)
{
    float d = fabsf(a - b);
    return d > 180.0f ? 360.0f - d : d;
}

/* 把角度差归一化到 (-180, 180] 度, 用于环形差值/残差. */
static float wrap_deg(float d)
{
    while (d > 180.0f) d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

/* α-β 跟踪器校正步 (环形安全).
 *
 * 预测: θ_pred = θ_smoothed + rate                     (rad)
 * 残差: e     = wrap(θ_meas - θ_pred)                  (rad)
 * 校正: θ_new = θ_pred + alpha * e
 *       rate  = rate  + beta  * e                      (rad/frame)
 *
 * beta 取 critical-damping 关系 beta = alpha^2 / (2 - alpha),
 * 使得匀速移动的声源由速率状态提前预测, 既有更低的跟踪滞后又不过冲.
 * 方向状态仍以单位矢量 (x,y) 表示以保证环绕 0/360 无缝.
 */
static void alpha_beta_track(doa_tracker_t *tracker, float angle_deg, float alpha)
{
    float measured = fmodf(angle_deg, 360.0f);
    if (measured < 0.0f) measured += 360.0f;
    float measured_rad = measured * M_PI / 180.0f;

    float pred_deg = tracker->held_angle +
        tracker->angular_rate * 180.0f / M_PI;
    float residual_deg = wrap_deg(measured - pred_deg);

    float beta = alpha * alpha / (2.0f - alpha);
    tracker->angular_rate += beta * residual_deg * (M_PI / 180.0f);

    float smoothed = pred_deg + alpha * residual_deg;
    if (smoothed < 0.0f) smoothed += 360.0f;
    if (smoothed >= 360.0f) smoothed -= 360.0f;
    float smoothed_rad = smoothed * M_PI / 180.0f;
    tracker->x = cosf(smoothed_rad);
    tracker->y = sinf(smoothed_rad);
    tracker->held_angle = smoothed;
    (void)measured_rad;
}

static float vector_angle(float x, float y)
{
    float angle = atan2f(y, x) * 180.0f / M_PI;
    return angle < 0.0f ? angle + 360.0f : angle;
}

static unsigned required_jump_frames(float movement_deg, float confidence)
{
    /* 对侧/镜像跳变: 用连续稳定帧数区分镜像(偶发)与真实切换(持续),
     * 中等置信 + 连续 5 帧; 置信过低返回 0 表示丢弃(锁原方向). */
    if (movement_deg > OPPOSITE_JUMP_DEG) {
        return confidence >= OPPOSITE_MIN_CONFIDENCE
            ? OPPOSITE_STABLE_FRAMES : 0U;
    }
    if (movement_deg > WIDE_JUMP_DEG) {
        return confidence >= WIDE_CONFIDENCE ? 3U : 5U;
    }
    if (movement_deg > MEDIUM_JUMP_DEG) {
        return confidence >= MEDIUM_CONFIDENCE ? 3U : 4U;
    }
    return confidence >= MEDIUM_CONFIDENCE ? REQUIRED_STABLE_FRAMES : 3U;
}

static bool accumulate_pending(doa_tracker_t *tracker, float angle_deg,
                               bool opposite_jump, bool high_confidence_jump,
                               unsigned required_frames)
{
    float radians = angle_deg * M_PI / 180.0f;
    float x = cosf(radians), y = sinf(radians);
    if (tracker->pending_count > 0) {
        float pending_angle = vector_angle(tracker->pending_x, tracker->pending_y);
        if (tracker->pending_opposite_jump != opposite_jump ||
            tracker->pending_high_confidence_jump != high_confidence_jump ||
            circular_distance(angle_deg, pending_angle) > PENDING_SPREAD_DEG) {
            tracker->pending_count = 0;
        }
    }
    if (tracker->pending_count == 0) {
        tracker->pending_x = x;
        tracker->pending_y = y;
        tracker->pending_count = 1;
        tracker->pending_opposite_jump = opposite_jump;
        tracker->pending_high_confidence_jump = high_confidence_jump;
    } else {
        tracker->pending_x += x;
        tracker->pending_y += y;
        tracker->pending_count++;
    }
    tracker->pending_misses = 0;
    return tracker->pending_count >= required_frames;
}

void doa_tracker_init(doa_tracker_t *tracker, float min_confidence)
{
    if (!tracker) return;
    memset(tracker, 0, sizeof(*tracker));
    tracker->min_confidence = min_confidence;
}

bool doa_tracker_update_transient(doa_tracker_t *tracker, float angle_deg,
                                  float confidence, bool candidate_valid,
                                  float *held_angle)
{
    if (!tracker || !held_angle) return false;

    if (tracker->transient_guard_frames > 0U) {
        tracker->transient_guard_frames--;
        *held_angle = tracker->held_angle;
        return false;
    }

    if (!candidate_valid || !isfinite(angle_deg) || !isfinite(confidence) ||
        confidence < tracker->min_confidence) {
        *held_angle = tracker->held_angle;
        return false;
    }

    float normalized = fmodf(angle_deg, 360.0f);
    if (normalized < 0.0f) normalized += 360.0f;
    /* 冷启动无历史方向可对照, 镜像伪影只能靠绝对置信把关 */
    if (!tracker->initialized && confidence < TRANSIENT_INITIAL_LOCK_MIN_CONF) {
        *held_angle = tracker->held_angle;
        return false;
    }
    /* 对侧镜像保护: 已锁定方向时, 瞬态角度跳到对侧(>120°)且置信不足,
     * 视为 GCC-PHAT 对称伪影, 保持原方向(避免响指亮对面). */
    if (tracker->initialized) {
        float movement = circular_distance(normalized, tracker->held_angle);
        if (movement > TRANSIENT_OPPOSITE_JUMP_DEG &&
            confidence < TRANSIENT_OPPOSITE_MIN_CONF) {
            *held_angle = tracker->held_angle;
            return false;
        }
    }
    float radians = normalized * M_PI / 180.0f;
    tracker->x = cosf(radians);
    tracker->y = sinf(radians);
    tracker->held_angle = normalized;
    tracker->angular_rate = 0.0f;
    tracker->pending_x = 0.0f;
    tracker->pending_y = 0.0f;
    tracker->pending_count = 0;
    tracker->pending_misses = 0;
    tracker->transient_guard_frames = TRANSIENT_GUARD_FRAMES;
    tracker->pending_high_confidence_jump = false;
    tracker->pending_opposite_jump = false;
    tracker->initialized = true;
    *held_angle = normalized;
    return true;
}

bool doa_tracker_update(doa_tracker_t *tracker, float angle_deg, float confidence,
                        bool candidate_valid, float *held_angle)
{
    if (!tracker || !held_angle) return false;
    bool transient_guard_active = tracker->transient_guard_frames > 0U;
    if (transient_guard_active) {
        tracker->transient_guard_frames--;
    }
    if (!candidate_valid || !isfinite(angle_deg) || !isfinite(confidence) ||
        confidence < tracker->min_confidence) {
        if (tracker->pending_count > 0) {
            if (tracker->pending_high_confidence_jump ||
                ++tracker->pending_misses > MAX_PENDING_MISSES) {
                tracker->pending_count = 0;
                tracker->pending_misses = 0;
                tracker->pending_high_confidence_jump = false;
                tracker->pending_opposite_jump = false;
            }
        }
        *held_angle = tracker->held_angle;
        return false;
    }

    float radians = angle_deg * M_PI / 180.0f;
    float x = cosf(radians), y = sinf(radians);
    if (!tracker->initialized) {
        if (!accumulate_pending(tracker, angle_deg, false, false,
                                REQUIRED_STABLE_FRAMES)) {
            *held_angle = tracker->held_angle;
            return false;
        }
        tracker->x = tracker->pending_x;
        tracker->y = tracker->pending_y;
        tracker->angular_rate = 0.0f;
        tracker->pending_count = 0;
        tracker->pending_high_confidence_jump = false;
        tracker->initialized = true;
    } else {
        float movement = circular_distance(angle_deg, tracker->held_angle);
        if (transient_guard_active && movement > LARGE_JUMP_DEG) {
            tracker->pending_count = 0;
            tracker->pending_misses = 0;
            tracker->pending_high_confidence_jump = false;
            tracker->pending_opposite_jump = false;
            *held_angle = tracker->held_angle;
            return false;
        }
        bool direct_jump = false;
        if (movement > LARGE_JUMP_DEG) {
            bool opposite_jump = movement > OPPOSITE_JUMP_DEG;
            /* 对侧/镜像跳变不享受高置信直通: 镜像伪影往往短促且置信不低,
             * 必须靠"连续稳定帧数"区分 (伪影 1~2 帧即消失, 真实切换持续),
             * 一律走 required_jump_frames 的 5 帧确认, 否则响指/反射会瞬间
             * 把方向翻到对面. */
            unsigned required;
            if (opposite_jump) {
                required = required_jump_frames(movement, confidence);
            } else if (confidence >= HIGH_CONFIDENCE_JUMP) {
                required = HIGH_CONFIDENCE_JUMP_FRAMES;
            } else {
                required = required_jump_frames(movement, confidence);
            }
            /* required==0: 置信过低无法确认跳变(镜像假峰或弱信号), 锁原方向 */
            if (required == 0U) {
                *held_angle = tracker->held_angle;
                return false;
            }
            /* 高置信严格模式只用于非对侧跳变 (对侧一律 5 帧连续确认,
             * 容忍少量无效帧, 否则真实切换在噪声下难以凑满连续帧) */
            bool high_confidence_jump =
                !opposite_jump && confidence >= HIGH_CONFIDENCE_JUMP;
            if (!accumulate_pending(tracker, angle_deg, opposite_jump,
                                    high_confidence_jump, required)) {
                *held_angle = tracker->held_angle;
                return false;
            }
            x = tracker->pending_x / (float)tracker->pending_count;
            y = tracker->pending_y / (float)tracker->pending_count;
            tracker->pending_count = 0;
            tracker->pending_misses = 0;
            tracker->pending_high_confidence_jump = false;
            tracker->pending_opposite_jump = false;
            /* 所有经确认的大跳变(含对侧)都直接跳到新方向并重置速率,
             * 避免真实声源切换被 α-β 一路拖成跑马灯. */
            tracker->x = x;
            tracker->y = y;
            tracker->angular_rate = 0.0f;
            direct_jump = true;
        } else {
            tracker->pending_count = 0;
            tracker->pending_misses = 0;
            tracker->pending_high_confidence_jump = false;
            tracker->pending_opposite_jump = false;
        }
        /* 任何经确认的大跳变都应重置速率状态: 音源位置发生了不连续变化,
         * 残留角速度会把后续帧错位预测到远处. 非跳变路径则由 α-β 自然收敛. */
        if (movement > LARGE_JUMP_DEG) {
            tracker->angular_rate = 0.0f;
        }
        if (!direct_jump) {
            /* 分档平滑: 静止/微动(<15°)强平滑滤 ±10° 抖动, 中速(15~30°)中等,
             * 快速(30~60°)较快跟踪. >60° 已走 direct jump 不经过这里. */
            float alpha;
            if (movement <= 15.0f) {
                alpha = 0.15f;
            } else if (movement <= 30.0f) {
                alpha = 0.28f;
            } else {
                alpha = confidence >= MEDIUM_CONFIDENCE ? 0.45f : 0.35f;
            }
            alpha_beta_track(tracker, angle_deg, alpha);
        }
    }
    tracker->held_angle = atan2f(tracker->y, tracker->x) * 180.0f / M_PI;
    if (tracker->held_angle < 0.0f) tracker->held_angle += 360.0f;
    *held_angle = tracker->held_angle;
    return true;
}
