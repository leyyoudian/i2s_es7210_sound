#include "gcc_phat.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MIC_COUNT 4
#define PAIR_COUNT 6
/* 2° 栅格: 48kHz 下 TDOA 分辨率足以支撑, 5° 栅格会成为角度误差下限 */
#define SRP_STEP_DEG 2
#define SRP_BIN_COUNT (360 / SRP_STEP_DEG)
/* 次峰搜索排除区 ~14° (2°/bin), 用于置信度计算 */
#define SRP_EXCLUSION_BINS 7
#define SRP_MIN_CONFIDENCE 0.025f
#define SRP_MIN_FREQ_HZ 1500.0f
/* 频带恢复基线 (鸟叫/响指等高频瞬态): 1500~6000Hz.
 * 鸟鸣主要能量 2~8kHz, 响指 1~8kHz, 都落在这个带内.
 * 注: 之前被改成 300~2000Hz 低频窄带 (为狗叫/人声设计), 导致
 * 鸟叫/响指/音乐的高频能量被硬切, GCC-PHAT 收不到相干信号. */
#define SRP_MAX_FREQ_HZ 6000.0f
#define PAIR_MIN_QUALITY 0.05f
/* 分数轮廓 EMA: 21.3ms 帧周期下 0.33 对应约 200ms 时间常数 (原 16kHz/64ms 帧 0.70).
 * 运行时可调: 持续声源 (音乐) 在反射房间中直接声峰稳定、反射峰随频谱游走,
 * 拉长积分自动偏向直接声; 瞬态/新声源用默认值保响应. */
#define SCORE_EMA_ALPHA 0.33f
static float s_score_ema_alpha = SCORE_EMA_ALPHA;
/* EMA 突变重置阈值: 30 bin × 2° = 60°, 与旧 5° 栅格下的 12 bin 等价 */
#define EMA_RESET_BIN_DISTANCE 30
/* 每通道自功率谱 EMA (相干性加权用): 0.25 @46.9fps ≈ 60ms 时间常数.
 * 稳态声源几帧内收敛 -> 加权生效; 突发瞬态首帧 γ 截顶为 1, 行为退化为
 * 纯 PHAT (不压制突发). */
#define AUTO_POW_EMA_ALPHA 0.25f

typedef struct {
    float re;
    float im;
} cpx_t;

static const char *TAG = "gcc_phat";
static const float s_mic_pos_m[MIC_COUNT][2] = {
    /* 40mm 半径十字阵列: MIC3(0°)/MIC4(90°)/MIC2(180°)/MIC1(270°) */
    {0.0f, -0.040f},
    {-0.040f, 0.0f},
    {0.040f, 0.0f},
    {0.0f, 0.040f},
};
static const uint8_t s_pairs[PAIR_COUNT][2] = {
    {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3},
};

static float s_window[GCC_FFT_SIZE];
static int s_bitrev[GCC_FFT_SIZE];
static cpx_t s_twiddle[GCC_FFT_SIZE / 2];
static cpx_t s_spectrum[MIC_COUNT][GCC_FFT_SIZE];
static cpx_t s_work[GCC_FFT_SIZE];
static float s_corr[PAIR_COUNT][GCC_FFT_SIZE];
static float s_pair_weight[PAIR_COUNT];
static float s_scores[SRP_BIN_COUNT];
static float s_score_ema[SRP_BIN_COUNT];
static bool s_score_ema_valid;
static int s_frame_len;
static bool s_initialized;
/* 每通道自功率谱 EMA (只存半谱): 相干性加权的分母. γ = |Sxy|/sqrt(Pi·Pj)
 * 对电平天然不敏感 (分子分母随增益同阶缩放), 远距/小声源时噪声主导的 bin
 * γ→0 被压掉, 相干 bin γ→1 保留 — 替代纯 PHAT 的"所有 bin 等权白化". */
static float s_auto_pow_ema[MIC_COUNT][GCC_FFT_SIZE / 2];

static void prepare_fft_tables(void)
{
    int bits = 0;
    for (int n = GCC_FFT_SIZE; n > 1; n >>= 1) bits++;
    for (int i = 0; i < GCC_FFT_SIZE; ++i) {
        int value = i;
        int reversed = 0;
        for (int b = 0; b < bits; ++b) {
            reversed = (reversed << 1) | (value & 1);
            value >>= 1;
        }
        s_bitrev[i] = reversed;
    }
    for (int i = 0; i < GCC_FFT_SIZE / 2; ++i) {
        float phase = -2.0f * M_PI * (float)i / (float)GCC_FFT_SIZE;
        s_twiddle[i].re = cosf(phase);
        s_twiddle[i].im = sinf(phase);
    }
}

static void fft_forward(cpx_t *x)
{
    for (int i = 0; i < GCC_FFT_SIZE; ++i) {
        if (s_bitrev[i] > i) {
            cpx_t tmp = x[i];
            x[i] = x[s_bitrev[i]];
            x[s_bitrev[i]] = tmp;
        }
    }
    for (int half = 1; half < GCC_FFT_SIZE; half <<= 1) {
        int tw_step = GCC_FFT_SIZE / (half << 1);
        for (int group = 0; group < GCC_FFT_SIZE; group += (half << 1)) {
            for (int k = 0; k < half; ++k) {
                cpx_t w = s_twiddle[k * tw_step];
                int a = group + k;
                int b = a + half;
                cpx_t product = {
                    w.re * x[b].re - w.im * x[b].im,
                    w.re * x[b].im + w.im * x[b].re,
                };
                cpx_t original = x[a];
                x[a].re = original.re + product.re;
                x[a].im = original.im + product.im;
                x[b].re = original.re - product.re;
                x[b].im = original.im - product.im;
            }
        }
    }
}

static void fft_inverse(cpx_t *x)
{
    for (int i = 0; i < GCC_FFT_SIZE; ++i) x[i].im = -x[i].im;
    fft_forward(x);
    const float scale = 1.0f / (float)GCC_FFT_SIZE;
    for (int i = 0; i < GCC_FFT_SIZE; ++i) {
        x[i].re *= scale;
        x[i].im *= -scale;
    }
}

static float correlation_at(const float *corr, float delay)
{
    int lower = (int)floorf(delay);
    float fraction = delay - (float)lower;
    int i0 = lower >= 0 ? lower : GCC_FFT_SIZE + lower;
    int upper = lower + 1;
    int i1 = upper >= 0 ? upper : GCC_FFT_SIZE + upper;
    i0 %= GCC_FFT_SIZE;
    i1 %= GCC_FFT_SIZE;
    return corr[i0] + fraction * (corr[i1] - corr[i0]);
}

static float pair_spacing_m(uint8_t pair_index)
{
    int i = s_pairs[pair_index][0], j = s_pairs[pair_index][1];
    float dx = s_mic_pos_m[i][0] - s_mic_pos_m[j][0];
    float dy = s_mic_pos_m[i][1] - s_mic_pos_m[j][1];
    return sqrtf(dx * dx + dy * dy);
}

static float pair_max_delay_samples(uint8_t pair_index, int sample_rate)
{
    return pair_spacing_m(pair_index) * (float)sample_rate / SOUND_SPEED_MPS;
}

static float pair_peak_quality(const float *corr, float max_delay_samples)
{
    int lag_limit = (int)ceilf(max_delay_samples) + 1;
    if (lag_limit < 1) lag_limit = 1;

    float peak = 0.0f;
    float side_sum = 0.0f;
    int side_count = 0;
    for (int lag = -lag_limit; lag <= lag_limit; ++lag) {
        float value = fabsf(correlation_at(corr, (float)lag));
        if (value > peak) peak = value;
        side_sum += value;
        side_count++;
    }
    if (peak <= 1e-7f || side_count <= 0) return 0.0f;

    float mean = side_sum / (float)side_count;
    float quality = (peak - mean) / (peak + 1e-6f);
    if (quality < 0.0f) quality = 0.0f;
    if (quality > 1.0f) quality = 1.0f;
    return quality;
}

static int find_best_bin(const float *scores, float *best_out)
{
    int best_bin = 0;
    float best = -INFINITY;
    for (int bin = 0; bin < SRP_BIN_COUNT; ++bin) {
        if (scores[bin] > best) {
            best = scores[bin];
            best_bin = bin;
        }
    }
    if (best_out) *best_out = best;
    return best_bin;
}

void gcc_phat_set_score_ema_alpha(float alpha)
{
    if (alpha >= 0.05f && alpha <= 1.0f) {
        s_score_ema_alpha = alpha;
    }
}

void gcc_phat_init(int frame_len)
{
    if (frame_len < 2) frame_len = 2;
    if (frame_len > GCC_FFT_SIZE) frame_len = GCC_FFT_SIZE;
    if (s_initialized && s_frame_len == frame_len) return;

    s_frame_len = frame_len;
    for (int i = 0; i < frame_len; ++i) {
        s_window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * (float)i /
                                         (float)(frame_len - 1)));
    }
    for (int i = frame_len; i < GCC_FFT_SIZE; ++i) s_window[i] = 0.0f;
    s_score_ema_valid = false;
    memset(s_auto_pow_ema, 0, sizeof(s_auto_pow_ema));
    prepare_fft_tables();
    s_initialized = true;
    ESP_LOGI(TAG, "4-mic SRP-PHAT initialized: frame=%d fft=%d grid=%d deg",
             frame_len, GCC_FFT_SIZE, SRP_STEP_DEG);
}

bool gcc_phat_estimate_4ch(const int16_t *mic[4], int n_samples,
                           int sample_rate, gcc_phat_result_t *out)
{
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    if (!mic || sample_rate <= 0 || n_samples < 2) return false;
    for (int ch = 0; ch < MIC_COUNT; ++ch) if (!mic[ch]) return false;
    if (!s_initialized || s_frame_len != (n_samples > GCC_FFT_SIZE ? GCC_FFT_SIZE : n_samples)) {
        gcc_phat_init(n_samples);
    }

    int count = n_samples < GCC_FFT_SIZE ? n_samples : GCC_FFT_SIZE;
    float channel_energy[MIC_COUNT] = {0.0f};
    float max_energy = 0.0f;
    for (int ch = 0; ch < MIC_COUNT; ++ch) {
        for (int i = 0; i < count; ++i) {
            float sample = (float)mic[ch][i] / 32768.0f;
            channel_energy[ch] += sample * sample;
            s_spectrum[ch][i].re = sample * s_window[i];
            s_spectrum[ch][i].im = 0.0f;
        }
        for (int i = count; i < GCC_FFT_SIZE; ++i) {
            s_spectrum[ch][i].re = 0.0f;
            s_spectrum[ch][i].im = 0.0f;
        }
        fft_forward(s_spectrum[ch]);
        for (int k = 0; k < GCC_FFT_SIZE / 2; ++k) {
            float p = s_spectrum[ch][k].re * s_spectrum[ch][k].re +
                      s_spectrum[ch][k].im * s_spectrum[ch][k].im;
            s_auto_pow_ema[ch][k] = AUTO_POW_EMA_ALPHA * p +
                (1.0f - AUTO_POW_EMA_ALPHA) * s_auto_pow_ema[ch][k];
        }
        if (channel_energy[ch] > max_energy) max_energy = channel_energy[ch];
    }
    if (max_energy < 1e-8) {
        s_score_ema_valid = false;
        memset(s_auto_pow_ema, 0, sizeof(s_auto_pow_ema));
        return false;
    }

    int active_mics = 0;
    for (int ch = 0; ch < MIC_COUNT; ++ch) {
        if (channel_energy[ch] >= max_energy * 0.02) {
            out->active_mic_mask |= (uint8_t)(1U << ch);
            active_mics++;
        }
    }
    if (active_mics < 3) {
        s_score_ema_valid = false;
        memset(s_auto_pow_ema, 0, sizeof(s_auto_pow_ema));
        return false;
    }

    for (int p = 0; p < PAIR_COUNT; ++p) {
        s_pair_weight[p] = 0.0f;
        uint8_t pair_mask = (uint8_t)((1U << s_pairs[p][0]) | (1U << s_pairs[p][1]));
        if ((out->active_mic_mask & pair_mask) != pair_mask) {
            memset(s_corr[p], 0, sizeof(s_corr[p]));
            continue;
        }
        out->active_pair_count++;
        const cpx_t *a = s_spectrum[s_pairs[p][0]];
        const cpx_t *b = s_spectrum[s_pairs[p][1]];
        for (int k = 0; k < GCC_FFT_SIZE; ++k) {
            float bin_hz = (k <= GCC_FFT_SIZE / 2)
                ? (float)k * (float)sample_rate / (float)GCC_FFT_SIZE
                : (float)(GCC_FFT_SIZE - k) * (float)sample_rate / (float)GCC_FFT_SIZE;
            if (bin_hz < SRP_MIN_FREQ_HZ || bin_hz > SRP_MAX_FREQ_HZ) {
                s_work[k].re = 0.0f;
                s_work[k].im = 0.0f;
                continue;
            }
            float re = a[k].re * b[k].re + a[k].im * b[k].im;
            float im = a[k].im * b[k].re - a[k].re * b[k].im;
            float magnitude = sqrtf(re * re + im * im);
            /* 相干性加权 (GCC-ML 的稳健版): 纯 PHAT 把噪声 bin 也白化到等幅,
             * 低 SNR (远距/小声) 时噪声 bin 的随机相位与信号 bin 平起平坐,
             * 峰被淹没. γ=|Sxy|/sqrt(Pi·Pj) 对真实相干 bin ≈1、噪声 bin ≈0,
             * 用 γ² 压掉噪声 bin, 只保留相干贡献. 突发首帧 P_EMA 尚未跟上,
             * γ 截顶为 1, 自动退化为 PHAT, 不伤瞬态. */
            int kh = (k <= GCC_FFT_SIZE / 2) ? k : GCC_FFT_SIZE - k;
            float gi = s_auto_pow_ema[s_pairs[p][0]][kh];
            float gj = s_auto_pow_ema[s_pairs[p][1]][kh];
            float coherence = magnitude / (sqrtf(gi * gj) + 1e-9f);
            if (coherence > 1.0f) coherence = 1.0f;
            float weight = coherence * coherence;
            float inv = magnitude > 1e-9f ? 1.0f / magnitude : 0.0f;
            s_work[k].re = re * inv * weight;
            s_work[k].im = im * inv * weight;
        }
        fft_inverse(s_work);
        for (int i = 0; i < GCC_FFT_SIZE; ++i) s_corr[p][i] = s_work[i].re;
        /* 质量权重 × 基线长度权重: 对角对 (50mm) 的 TDOA-角度斜率是相邻对 (35mm)
         * 的 1.41 倍, 方向分辨能力更强, 应在 SRP 求和中占更大比重 */
        s_pair_weight[p] = pair_peak_quality(
            s_corr[p], pair_max_delay_samples((uint8_t)p, sample_rate)) *
            (pair_spacing_m((uint8_t)p) / (MIC_ARRAY_RADIUS_MM * 2.0f / 1000.0f));
    }

    for (int bin = 0; bin < SRP_BIN_COUNT; ++bin) {
        float angle = (float)(bin * SRP_STEP_DEG) * M_PI / 180.0f;
        float ux = cosf(angle), uy = sinf(angle);
        float score = 0.0f;
        float weight_sum = 0.0f;
        for (int p = 0; p < PAIR_COUNT; ++p) {
            uint8_t pair_mask = (uint8_t)((1U << s_pairs[p][0]) | (1U << s_pairs[p][1]));
            if ((out->active_mic_mask & pair_mask) != pair_mask) continue;
            if (s_pair_weight[p] < PAIR_MIN_QUALITY) continue;
            int i = s_pairs[p][0], j = s_pairs[p][1];
            float dx = s_mic_pos_m[i][0] - s_mic_pos_m[j][0];
            float dy = s_mic_pos_m[i][1] - s_mic_pos_m[j][1];
            float delay = -(dx * ux + dy * uy) * (float)sample_rate / SOUND_SPEED_MPS;
            score += s_pair_weight[p] * correlation_at(s_corr[p], delay);
            weight_sum += s_pair_weight[p];
        }
        if (weight_sum > 0.0f) {
            score /= weight_sum;
        } else {
            score = -INFINITY;
        }
        s_scores[bin] = score;
    }

    float raw_best = -INFINITY;
    int raw_best_bin = find_best_bin(s_scores, &raw_best);
    if (!isfinite(raw_best)) {
        s_score_ema_valid = false;
        return false;
    }
    if (!s_score_ema_valid) {
        memcpy(s_score_ema, s_scores, sizeof(s_scores));
        s_score_ema_valid = true;
    } else {
        float ema_best = -INFINITY;
        int ema_best_bin = find_best_bin(s_score_ema, &ema_best);
        int bin_distance = abs(raw_best_bin - ema_best_bin);
        if (bin_distance > SRP_BIN_COUNT / 2) bin_distance = SRP_BIN_COUNT - bin_distance;
        if (bin_distance > EMA_RESET_BIN_DISTANCE && raw_best > 0.25f) {
            memcpy(s_score_ema, s_scores, sizeof(s_scores));
        } else {
            for (int bin = 0; bin < SRP_BIN_COUNT; ++bin) {
                s_score_ema[bin] = s_score_ema_alpha * s_scores[bin] +
                    (1.0f - s_score_ema_alpha) * s_score_ema[bin];
            }
        }
    }

    float best = -INFINITY;
    int best_bin = find_best_bin(s_score_ema, &best);

    float second = -INFINITY;
    for (int bin = 0; bin < SRP_BIN_COUNT; ++bin) {
        int distance = abs(bin - best_bin);
        if (distance > SRP_BIN_COUNT / 2) distance = SRP_BIN_COUNT - distance;
        if (distance > SRP_EXCLUSION_BINS && s_score_ema[bin] > second) second = s_score_ema[bin];
    }

    int prev = (best_bin + SRP_BIN_COUNT - 1) % SRP_BIN_COUNT;
    int next = (best_bin + 1) % SRP_BIN_COUNT;
    float denom = 2.0f * (2.0f * best - s_score_ema[prev] - s_score_ema[next]);
    float offset = 0.0f;
    if (fabsf(denom) > 1e-7f) {
        offset = (s_score_ema[next] - s_score_ema[prev]) / denom;
        if (offset > 0.5f) offset = 0.5f;
        if (offset < -0.5f) offset = -0.5f;
    }

    float angle = ((float)best_bin + offset) * (float)SRP_STEP_DEG;
    if (angle < 0.0f) angle += 360.0f;
    if (angle >= 360.0f) angle -= 360.0f;
    float confidence = (best - second) / (fabsf(best) + 1e-6f);
    /* 镜像比: EMA 谱上与本峰相隔 180° 的 bin 分数. 中心对称阵列在混叠频段
     * (对径基线 80mm -> 2.14kHz 以上) 会生成接近真实峰的镜像峰, 室内反射
     * 也会抬高对侧; 该比值供上层判断本帧方向是否含糊. */
    int mirror_bin = (best_bin + SRP_BIN_COUNT / 2) % SRP_BIN_COUNT;
    float mirror = s_score_ema[mirror_bin];
    float mirror_ratio = mirror / (best + 1e-6f);
    if (mirror_ratio < 0.0f) mirror_ratio = 0.0f;

    out->angle_deg = angle;
    out->score = best;
    out->second_score = second;
    out->confidence = confidence;
    out->mirror_ratio = mirror_ratio;
    out->valid = isfinite(angle) && isfinite(confidence) && best > 0.0f &&
                 confidence >= SRP_MIN_CONFIDENCE;
    return out->valid;
}
