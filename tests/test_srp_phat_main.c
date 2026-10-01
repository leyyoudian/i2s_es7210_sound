#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "gcc_phat.h"

/* 与固件几何/配置保持一致:
 *  - 阵列半径 40mm (MIC_ARRAY_RADIUS_MM), 与 gcc_phat.c 的 s_mic_pos_m 相同
 *  - 48kHz/1024 (当前固件配置, GCC_FFT_SIZE=1024) 与 16kHz/1024 (旧配置) */
#define N_MAX 2048

static uint32_t rng_state = 1;
static int16_t base_signal[N_MAX + 32];
static int16_t mic_data[4][N_MAX];

static float circular_error(float a, float b)
{
    float d = fabsf(a - b);
    return d > 180.0f ? 360.0f - d : d;
}

static void make_signal(float angle_deg, int n, int fs)
{
    const float positions[4][2] = {
        {0.0f, -0.040f}, {-0.040f, 0.0f},
        {0.040f, 0.0f}, {0.0f, 0.040f},
    };
    const float a = angle_deg * 3.14159265358979323846f / 180.0f;
    const float ux = cosf(a), uy = sinf(a);

    for (int i = 0; i < n + 32; ++i) {
        rng_state = rng_state * 1664525u + 1013904223u;
        base_signal[i] = (int16_t)(((rng_state >> 16) & 0xffffu) - 32768);
    }

    for (int ch = 0; ch < 4; ++ch) {
        int delay = (int)lrintf(-(positions[ch][0] * ux + positions[ch][1] * uy)
                                * (float)fs / 343.0f);
        for (int i = 0; i < n; ++i) {
            mic_data[ch][i] = base_signal[i + 16 - delay];
        }
    }
}

static int run_case_set(int fs, int n)
{
    static const float cases[] = {0.0f, 45.0f, 90.0f, 180.0f, 270.0f, 315.0f};
    const int16_t *mics[4] = {mic_data[0], mic_data[1], mic_data[2], mic_data[3]};

    gcc_phat_init(n);
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        gcc_phat_result_t result = {0};
        /* 固件中 SRP 分数轮廓带帧间 EMA, 与真实连续帧一致, 每个角度跑 3 帧
         * 让 EMA 收敛后再判读 (单帧判读会被用例间 45° 突跳的过渡态污染) */
        for (int frame = 0; frame < 3; ++frame) {
            make_signal(cases[i], n, fs);
            if (!gcc_phat_estimate_4ch(mics, n, fs, &result) || !result.valid) {
                fprintf(stderr, "[fs=%d] %.1f degrees was rejected\n", fs, cases[i]);
                return 1;
            }
        }
        if (circular_error(result.angle_deg, cases[i]) > 10.0f) {
            fprintf(stderr, "[fs=%d] expected %.1f, got %.1f (confidence %.3f)\n",
                    fs, cases[i], result.angle_deg, result.confidence);
            return 1;
        }
        /* 干净信号下对侧镜像峰应远低于主峰 (镜像含糊判定门限 0.60) */
        if (result.mirror_ratio >= 0.60f) {
            fprintf(stderr, "[fs=%d] %.1f degrees mirror_ratio %.3f too high\n",
                    fs, cases[i], result.mirror_ratio);
            return 5;
        }
    }

    make_signal(90.0f, n, fs);
    for (int i = 0; i < n; ++i) mic_data[0][i] = 0;
    gcc_phat_result_t degraded = {0};
    for (int frame = 0; frame < 3; ++frame) {
        if (!gcc_phat_estimate_4ch(mics, n, fs, &degraded) || !degraded.valid) {
            fprintf(stderr, "[fs=%d] three-microphone fallback was rejected\n", fs);
            return 2;
        }
    }
    if (degraded.active_mic_mask != 0x0e || degraded.active_pair_count != 3) {
        fprintf(stderr, "[fs=%d] bad health mask=0x%x pairs=%u\n",
                fs, degraded.active_mic_mask, degraded.active_pair_count);
        return 3;
    }
    if (circular_error(degraded.angle_deg, 90.0f) > 10.0f) {
        fprintf(stderr, "[fs=%d] three-microphone expected 90, got %.1f\n",
                fs, degraded.angle_deg);
        return 4;
    }
    return 0;
}

int main(void)
{
    int rc = run_case_set(48000, 1024);
    if (rc != 0) {
        return rc;
    }
    return run_case_set(16000, 1024);
}
