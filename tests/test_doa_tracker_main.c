#include <math.h>
#include <stdio.h>

#include "doa_tracker.h"

int main(void)
{
    doa_tracker_t tracker;
    float held = 0.0f;
    doa_tracker_init(&tracker, 0.15f);

    if (doa_tracker_update(&tracker, 359.0f, 0.8f, true, &held)) return 1;
    if (!doa_tracker_update(&tracker, 1.0f, 0.8f, true, &held)) return 2;
    if (held > 15.0f && held < 345.0f) {
        fprintf(stderr, "wrap smoothing failed: %.1f\n", held);
        return 4;
    }
    const float before = held;
    if (doa_tracker_update(&tracker, 180.0f, 0.01f, false, &held)) return 5;
    if (fabsf(held - before) > 0.01f) {
        fprintf(stderr, "invalid frame moved angle: %.1f -> %.1f\n", before, held);
        return 6;
    }

    if (doa_tracker_update(&tracker, 70.0f, 0.20f, true, &held)) return 7;
    if (fabsf(held - before) > 0.01f) return 8;
    if (doa_tracker_update(&tracker, 72.0f, 0.01f, false, &held)) return 9;
    if (doa_tracker_update(&tracker, 71.0f, 0.20f, true, &held)) return 10;
    if (doa_tracker_update(&tracker, 70.0f, 0.20f, true, &held)) return 24;
    if (!doa_tracker_update(&tracker, 71.0f, 0.20f, true, &held)) return 25;

    doa_tracker_init(&tracker, 0.08f);
    if (doa_tracker_update(&tracker, 134.0f, 0.24f, true, &held)) return 15;
    if (!doa_tracker_update(&tracker, 135.0f, 0.24f, true, &held)) return 16;
    const float stable = held;
    /* 对侧低置信跳变: 连续 5 帧 (OPPOSITE_STABLE_FRAMES) 才确认 */
    if (doa_tracker_update(&tracker, 331.0f, 0.21f, true, &held)) return 17;
    if (doa_tracker_update(&tracker, 333.0f, 0.21f, true, &held)) return 18;
    if (fabsf(held - stable) > 1.0f) {
        fprintf(stderr, "opposite low-confidence jump moved angle: %.1f -> %.1f\n",
                stable, held);
        return 19;
    }
    if (doa_tracker_update(&tracker, 332.0f, 0.30f, true, &held)) return 20;
    if (doa_tracker_update(&tracker, 334.0f, 0.30f, true, &held)) return 21;
    if (!doa_tracker_update(&tracker, 333.0f, 0.30f, true, &held)) return 22;
    if (held < 300.0f) {
        fprintf(stderr, "confirmed opposite jump missed: %.1f\n", held);
        return 23;
    }

    doa_tracker_init(&tracker, 0.15f);
    if (doa_tracker_update(&tracker, 0.0f, 0.8f, true, &held)) return 11;
    if (!doa_tracker_update(&tracker, 1.0f, 0.8f, true, &held)) return 12;
    const float high_stable = held;
    /* 对侧镜像伪影即使高置信也不得 2 帧直通, 必须连续 5 帧稳定:
     * 伪影 1~2 帧即消失, 真实切换持续多帧 */
    if (doa_tracker_update(&tracker, 180.0f, 0.36f, true, &held)) return 13;
    if (fabsf(held - high_stable) > 1.0f) {
        fprintf(stderr, "single high-confidence outlier moved angle: %.1f -> %.1f\n",
                high_stable, held);
        return 14;
    }
    if (doa_tracker_update(&tracker, 0.0f, 0.0f, false, &held)) return 26;
    if (fabsf(held - high_stable) > 1.0f) return 27;
    if (doa_tracker_update(&tracker, 178.0f, 0.36f, true, &held)) return 28;
    if (doa_tracker_update(&tracker, 180.0f, 0.36f, true, &held)) return 40;
    if (doa_tracker_update(&tracker, 179.0f, 0.36f, true, &held)) return 41;
    if (!doa_tracker_update(&tracker, 180.0f, 0.36f, true, &held)) return 29;
    if (fabsf(held - 179.4f) > 5.0f) {
        fprintf(stderr, "confirmed opposite jump missed: %.1f\n", held);
        return 30;
    }

    doa_tracker_init(&tracker, 0.15f);
    /* 冷启动瞬态锁定需要 TRANSIENT_INITIAL_LOCK_MIN_CONF 以上置信 */
    if (doa_tracker_update_transient(&tracker, 45.0f, 0.30f, true, &held)) {
        return 42;
    }
    if (!doa_tracker_update_transient(&tracker, 45.0f, 0.40f, true, &held)) {
        return 43;
    }

    doa_tracker_init(&tracker, 0.15f);
    if (doa_tracker_update(&tracker, 0.0f, 0.8f, true, &held)) return 31;
    if (!doa_tracker_update(&tracker, 1.0f, 0.8f, true, &held)) return 32;
    if (!doa_tracker_update_transient(
            &tracker, 270.0f, 0.36f, true, &held)) return 33;
    if (fabsf(held - 270.0f) > 0.1f) {
        fprintf(stderr, "transient did not update immediately: %.1f\n", held);
        return 34;
    }
    for (int i = 0; i < 6; ++i) {
        if (doa_tracker_update(&tracker, 90.0f, 0.8f, true, &held)) return 35;
        if (fabsf(held - 270.0f) > 0.1f) {
            fprintf(stderr, "reflection moved guarded transient on frame %d: %.1f\n",
                    i + 1, held);
            return 36;
        }
    }
    /* 保护期后对侧(180°)切换同样需要 5 帧连续确认 */
    if (doa_tracker_update(&tracker, 90.0f, 0.8f, true, &held)) return 37;
    if (doa_tracker_update(&tracker, 90.0f, 0.8f, true, &held)) return 44;
    if (doa_tracker_update(&tracker, 90.0f, 0.8f, true, &held)) return 45;
    if (doa_tracker_update(&tracker, 90.0f, 0.8f, true, &held)) return 46;
    if (!doa_tracker_update(&tracker, 90.0f, 0.8f, true, &held)) return 38;
    if (fabsf(held - 90.0f) > 0.1f) {
        fprintf(stderr, "ordinary tracking did not resume after guard: %.1f\n", held);
        return 39;
    }
    return 0;
}
