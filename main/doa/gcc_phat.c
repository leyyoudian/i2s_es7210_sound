#include "gcc_phat.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

#ifdef ESP_PLATFORM
#include "dsps_fft2r.h"
#define FFT_ALIGN __attribute__((aligned(16)))
#else
#define FFT_ALIGN
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MIC_COUNT                          4                      /* 物理麦克风数，与s_mic_pos_m一一对应 */
#define PAIR_COUNT                         6                      /* 四麦无序组合数4*3/2=6 */
/* 2° 栅格: 48kHz 下 TDOA 分辨率足以支撑, 5° 栅格会成为角度误差下限 */
#define SRP_STEP_DEG                       2                      /* 角搜索步长度数，越小搜索越密但不等于绝对精度 */
#define SRP_BIN_COUNT                      (360 / SRP_STEP_DEG)   /* 360度搜索格数，当前180 */
/* Compare independent angular peaks outside a 30-degree main-lobe region. */
#define SRP_EXCLUSION_BINS                 15                     /* 主瓣排除半宽15*2=30度，避免把峰肩当独立声源 */
#define SRP_MIN_CONFIDENCE                 0.025f                 /* 底层有效性最低峰区分度，上层还有RMS/镜像检查 */
#define SRP_MIN_FREQ_HZ                    300.0f                 /* 定位频带下限Hz，保留语音低频时差信息 */
/* Keep low-frequency time-delay cues as well as bird/transient harmonics. */
#define SRP_MAX_FREQ_HZ                    6000.0f                /* 定位频带上限Hz，与前级低通一致 */
#define PAIR_MIN_QUALITY                   0.05f                  /* 麦对相关峰质量下限，过差麦对不参与空间求和 */
/* 21.3 ms frames: alpha=.33 gives a 53 ms time constant; .15 gives 131 ms. */
#define SCORE_EMA_ALPHA                    0.33f                  /* 空间图EMA默认系数，21.33ms帧下约53ms时间常数 */
static float s_score_ema_alpha = SCORE_EMA_ALPHA;
/* 每通道自功率谱 EMA (相干性加权用): 0.25 @46.9fps ≈ 74ms 时间常数.
 * 稳态声源几帧内收敛 -> 加权生效; 突发瞬态首帧 γ 截顶为 1, 行为退化为
 * 纯 PHAT (不压制突发). */
#define AUTO_POW_EMA_ALPHA                 0.25f                  /* 复数互谱/自功率EMA，约74ms时间常数 */

/* 以下只用于提取多方向灯效候选，不改动上面已调好的主方向门槛。
 * 反射也可能构成第二峰，故必须同时满足空间分离、谷深、麦对支持、
 * 主峰比例，并由上层持续确认；不能把 second_score 直接画成第二音源。 */
#define DISPLAY_PEAK_MIN_SEPARATION_DEG 60.0f /* 太近的双源本小阵列难稳定分辨 */
#define DISPLAY_PEAK_MIN_SCORE .012f           /* 归一化相关分数绝对下限 */
#define DISPLAY_PEAK_MIN_RATIO .55f            /* 次峰至少达到主峰 55% */
#define DISPLAY_PEAK_MIN_PROMINENCE .12f       /* ±20°两侧均须有谷，拒绝宽主瓣 */
#define DISPLAY_PEAK_MIN_PAIR_SUPPORT 4        /* 六个麦对中至少四对正向支持 */
#define DISPLAY_PEAK_MAX_MIRROR .75f           /* 对侧强近似峰先视为含糊 */
#define DISPLAY_PEAK_OPPOSITE_PROMINENCE .25f  /* 真实对向双源仅在两个尖峰都清晰时保留 */

typedef struct {
    float re;
    float im;
} cpx_t;

static const char *TAG = "gcc_phat";
static const float s_mic_pos_m[MIC_COUNT][2] = {
    /* 40mm 半径十字阵列: MIC3(0°)/MIC4(90°)/MIC2(180°)/MIC1(270°) */
    {0.0f, -MIC_ARRAY_RADIUS_MM / 1000.0f},
    {-MIC_ARRAY_RADIUS_MM / 1000.0f, 0.0f},
    {MIC_ARRAY_RADIUS_MM / 1000.0f, 0.0f},
    {0.0f, MIC_ARRAY_RADIUS_MM / 1000.0f},
};
static const uint8_t s_pairs[PAIR_COUNT][2] = {
    {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3},
};

static float s_window[GCC_FFT_SIZE];
static int s_bitrev[GCC_FFT_SIZE];
static cpx_t s_twiddle[GCC_FFT_SIZE / 2] FFT_ALIGN;
static cpx_t s_spectrum[MIC_COUNT][GCC_FFT_SIZE] FFT_ALIGN;
static cpx_t s_work[GCC_FFT_SIZE] FFT_ALIGN;
static float s_corr[PAIR_COUNT][GCC_FFT_SIZE];
static float s_pair_weight[PAIR_COUNT];
static float s_scores[SRP_BIN_COUNT];
static float s_instant_scores[SRP_BIN_COUNT];
static float s_score_ema[SRP_BIN_COUNT];
static bool s_score_ema_valid;
static float s_zero_delay_ema;
static int s_frame_len;
static bool s_initialized;
static uint8_t s_active_mic_mask;
/* 每通道自功率谱 EMA (只存半谱): 相干性加权的分母. γ = |Sxy|/sqrt(Pi·Pj)
 * 对电平天然不敏感 (分子分母随增益同阶缩放), 远距/小声源时噪声主导的 bin
 * γ→0 被压掉, 相干 bin γ→1 保留 — 替代纯 PHAT 的"所有 bin 等权白化". */
static float s_auto_pow_ema[MIC_COUNT][GCC_FFT_SIZE / 2 + 1];
static cpx_t s_cross_ema[PAIR_COUNT][GCC_FFT_SIZE / 2 + 1];
static float s_steering_delay[SRP_BIN_COUNT][PAIR_COUNT];
static int s_steering_rate;
static float s_frequency_weight[GCC_FFT_SIZE / 2 + 1];
static float s_noise_power[MIC_COUNT][GCC_FFT_SIZE / 2 + 1];
static bool s_noise_learning;
static bool s_noise_initialized;
static bool s_spectral_history_valid;
static bool s_instant_enabled = true;

void gcc_phat_set_instant_enabled(bool enabled)
{
    s_instant_enabled = enabled;
}

void gcc_phat_set_noise_learning(bool enabled)
{
    s_noise_learning = enabled;
}

static float foreground_weight(int pair, int k, float pi, float pj)
{
    if (!s_noise_initialized) return 1.0f;
    float wi = fmaxf(0.0f, 1.0f - s_noise_power[s_pairs[pair][0]][k] / (pi + 1e-12f));
    float wj = fmaxf(0.0f, 1.0f - s_noise_power[s_pairs[pair][1]][k] / (pj + 1e-12f));
    return sqrtf(wi * wj);
}

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
#ifdef ESP_PLATFORM
    /* ESP-DSP uses a positive-sine, bit-reversed half-size twiddle table. */
    for (int i = 0; i < GCC_FFT_SIZE / 2; ++i) s_twiddle[i].im = -s_twiddle[i].im;
    for (int i = 0; i < GCC_FFT_SIZE / 2; ++i) {
        int j = s_bitrev[i] >> 1;
        if (j > i) {
            cpx_t tmp = s_twiddle[i];
            s_twiddle[i] = s_twiddle[j];
            s_twiddle[j] = tmp;
        }
    }
#endif
}

static void fft_forward(cpx_t *x)
{
#ifdef ESP_PLATFORM
    dsps_fft2r_fc32_aes3_((float *)x, GCC_FFT_SIZE, (float *)s_twiddle);
#endif
    for (int i = 0; i < GCC_FFT_SIZE; ++i) {
        if (s_bitrev[i] > i) {
            cpx_t tmp = x[i];
            x[i] = x[s_bitrev[i]];
            x[s_bitrev[i]] = tmp;
        }
    }
#ifndef ESP_PLATFORM
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
#endif
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
    /* Delays are bounded by the microphone aperture. Truncation plus this
     * correction is floor() for either sign, without a libm call per steering
     * sample on the ESP32-S3. */
    int lower = (int)delay;
    if ((float)lower > delay) --lower;
    float fraction = delay - (float)lower;
    int i0 = lower >= 0 ? lower : GCC_FFT_SIZE + lower;
    int upper = lower + 1;
    int i1 = upper >= 0 ? upper : GCC_FFT_SIZE + upper;
    i0 %= GCC_FFT_SIZE;
    i1 %= GCC_FFT_SIZE;
    if (fraction == 0.0f) return corr[i0];
    /* The correlation is band-limited; linear interpolation creates cusps
     * at integer sample delays and biases the angular maximum toward them.
     * Four-point cubic interpolation keeps sub-sample steering smooth. */
    float ym = corr[(i0 + GCC_FFT_SIZE - 1) % GCC_FFT_SIZE];
    float y0 = corr[i0], y1 = corr[i1];
    float yp = corr[(i1 + 1) % GCC_FFT_SIZE];
    return y0 + .5f * fraction * (y1 - ym + fraction *
        (2.f * ym - 5.f * y0 + 4.f * y1 - yp + fraction *
         (3.f * (y0 - y1) + yp - ym)));
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

static float display_directional_rms(const int16_t *mic[4], int count,
                                      int sample_rate, float angle_deg,
                                      uint8_t mask)
{
    /* 延时求和波束：x_ch(t + arrival_ch) 对齐同一个声源。只计算候选方向，
     * 不为整张角度图额外做 FFT。结果是带内接收强度，仍会混入反射/其它源。 */
    float radians = angle_deg * M_PI / 180.0f;
    int delay_int[4];
    float delay_fraction[4];
    int channels = 0;
    int guard = (int)ceilf(MIC_ARRAY_RADIUS_MM / 1000.0f * sample_rate / SOUND_SPEED_MPS) + 2;
    for (int ch = 0; ch < 4; ++ch) {
        float delay = -(s_mic_pos_m[ch][0] * cosf(radians) +
                        s_mic_pos_m[ch][1] * sinf(radians)) * sample_rate / SOUND_SPEED_MPS;
        delay_int[ch] = (int)floorf(delay);
        delay_fraction[ch] = delay - delay_int[ch];
        if (mask & (1U << ch)) channels++;
    }
    if (channels < 3 || count <= guard * 2) return 0;
    float energy = 0;
    for (int n = guard; n < count - guard; ++n) {
        float sample = 0;
        for (int ch = 0; ch < 4; ++ch) if (mask & (1U << ch)) {
            int i = n + delay_int[ch];
            sample += mic[ch][i] + delay_fraction[ch] * ((float)mic[ch][i + 1] - mic[ch][i]);
        }
        sample /= channels * 32768.0f;
        energy += sample * sample;
    }
    return sqrtf(energy / (count - guard * 2));
}

static void collect_display_peaks(const int16_t *mic[4], int count,
                                  int sample_rate, gcc_phat_result_t *out)
{
    float best;
    find_best_bin(s_score_ema, &best);
    if (!isfinite(best) || best < DISPLAY_PEAK_MIN_SCORE) return;
    int selected[GCC_MAX_DISPLAY_PEAKS] = {-1, -1};
    for (int rank = 0; rank < GCC_MAX_DISPLAY_PEAKS; ++rank) {
        float selected_score = -INFINITY, selected_prominence = 0;
        int selected_bin = -1;
        for (int bin = 0; bin < SRP_BIN_COUNT; ++bin) {
            float peak = s_score_ema[bin];
            if (peak < DISPLAY_PEAK_MIN_SCORE || peak < best * DISPLAY_PEAK_MIN_RATIO) continue;
            if (peak < s_score_ema[(bin + SRP_BIN_COUNT - 1) % SRP_BIN_COUNT] ||
                peak <= s_score_ema[(bin + 1) % SRP_BIN_COUNT]) continue;
            bool separate = true;
            for (int j = 0; j < rank; ++j) {
                int d = abs(bin - selected[j]);
                if (d > SRP_BIN_COUNT / 2) d = SRP_BIN_COUNT - d;
                if (d * SRP_STEP_DEG < DISPLAY_PEAK_MIN_SEPARATION_DEG) separate = false;
            }
            if (!separate) continue;
            float left = peak, right = peak;
            for (int d = 1; d <= 10; ++d) { /* 10×2°，固定物理角度宽度 */
                left = fminf(left, s_score_ema[(bin + SRP_BIN_COUNT - d) % SRP_BIN_COUNT]);
                right = fminf(right, s_score_ema[(bin + d) % SRP_BIN_COUNT]);
            }
            float prominence = (peak - fmaxf(left, right)) / (fabsf(peak) + 1e-6f);
            if (prominence < DISPLAY_PEAK_MIN_PROMINENCE) continue;
            float mirror = s_score_ema[(bin + SRP_BIN_COUNT / 2) % SRP_BIN_COUNT];
            if (mirror > peak * DISPLAY_PEAK_MAX_MIRROR &&
                prominence < DISPLAY_PEAK_OPPOSITE_PROMINENCE) continue;
            unsigned support = 0;
            for (int p = 0; p < PAIR_COUNT; ++p) {
                if (s_pair_weight[p] >= PAIR_MIN_QUALITY &&
                    correlation_at(s_corr[p], s_steering_delay[bin][p]) > .005f) support++;
            }
            if (support < DISPLAY_PEAK_MIN_PAIR_SUPPORT) continue;
            if (peak > selected_score) {
                selected_bin = bin; selected_score = peak; selected_prominence = prominence;
            }
        }
        if (selected_bin < 0) break;
        selected[rank] = selected_bin;
        gcc_phat_peak_t *peak = &out->display_peaks[out->display_peak_count++];
        peak->angle_deg = (float)(selected_bin * SRP_STEP_DEG);
        peak->score = selected_score;
        peak->prominence = selected_prominence;
    }
    /* 单方向沿用main已有的带内RMS，不重复做波束计算；只给真正双峰计算
     * 各自的接收强度，减少已验收单声源路径的额外CPU负载。 */
    if (out->display_peak_count == 2) {
        for (int i = 0; i < 2; ++i) {
            out->display_peaks[i].directional_rms = display_directional_rms(
                mic, count, sample_rate, out->display_peaks[i].angle_deg, out->active_mic_mask);
        }
    }
}

void gcc_phat_set_score_ema_alpha(float alpha)
{
    if (alpha >= 0.05f && alpha <= 1.0f) {
        s_score_ema_alpha = alpha;
    }
}

void gcc_phat_reset_history(void)
{
    s_score_ema_valid = false;
    s_spectral_history_valid = false;
    s_active_mic_mask = 0;
    memset(s_auto_pow_ema, 0, sizeof(s_auto_pow_ema));
    memset(s_cross_ema, 0, sizeof(s_cross_ema));
}

static void measure_peak(const float *scores, float *angle_out,
                         float *score_out, float *second_out,
                         float *confidence_out, float *mirror_out)
{
    float best;
    int bin = find_best_bin(scores, &best);
    float second = 0.0f;
    for (int k = 0; k < SRP_BIN_COUNT; ++k) {
        int distance = abs(k - bin);
        if (distance > SRP_BIN_COUNT / 2) distance = SRP_BIN_COUNT - distance;
        int prev_k = (k + SRP_BIN_COUNT - 1) % SRP_BIN_COUNT;
        int next_k = (k + 1) % SRP_BIN_COUNT;
        /* Compare competing peaks, not a broad main lobe's shoulder. The
         * latter wrongly labels useful low-frequency speech as ambiguous. */
        if (distance > SRP_EXCLUSION_BINS && scores[k] >= scores[prev_k] &&
            scores[k] > scores[next_k] && scores[k] > second) second = scores[k];
    }
    int prev = (bin + SRP_BIN_COUNT - 1) % SRP_BIN_COUNT;
    int next = (bin + 1) % SRP_BIN_COUNT;
    float denom = 2.0f * (2.0f * best - scores[prev] - scores[next]);
    float offset = fabsf(denom) > 1e-7f ? (scores[next] - scores[prev]) / denom : 0.0f;
    offset = fmaxf(-0.5f, fminf(0.5f, offset));
    *angle_out = fmodf(((float)bin + offset) * SRP_STEP_DEG + 360.0f, 360.0f);
    *score_out = best;
    *second_out = second;
    *confidence_out = (best - second) / (fabsf(best) + 1e-6f);
    *mirror_out = fmaxf(0.0f, scores[(bin + SRP_BIN_COUNT / 2) % SRP_BIN_COUNT] /
                                  (best + 1e-6f));
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
    gcc_phat_reset_history();
    prepare_fft_tables();
    /* Verify backend sign, bin order and inverse scaling on the real chip. */
    memset(s_work, 0, sizeof(s_work));
    s_work[1].re = 1.0f;
    fft_forward(s_work);
    for (int k = 0; k < GCC_FFT_SIZE; ++k) {
        float phase = -2.0f * M_PI * k / GCC_FFT_SIZE;
        if (fabsf(s_work[k].re - cosf(phase)) > .001f ||
            fabsf(s_work[k].im - sinf(phase)) > .001f) {
            ESP_LOGE(TAG, "FFT backend self-check failed at bin %d", k);
            s_initialized = false;
            return;
        }
    }
    fft_inverse(s_work);
    if (fabsf(s_work[1].re - 1.0f) > .001f) {
        ESP_LOGE(TAG, "FFT inverse self-check failed");
        s_initialized = false;
        return;
    }
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
    if (!s_initialized) return false;

    int count = n_samples < GCC_FFT_SIZE ? n_samples : GCC_FFT_SIZE;
    if (s_steering_rate != sample_rate) {
        for (int bin = 0; bin < SRP_BIN_COUNT; ++bin) {
            float radians = bin * SRP_STEP_DEG * M_PI / 180.0f;
            float ux = cosf(radians), uy = sinf(radians);
            for (int p = 0; p < PAIR_COUNT; ++p) {
                int i = s_pairs[p][0], j = s_pairs[p][1];
                s_steering_delay[bin][p] =
                    -((s_mic_pos_m[i][0] - s_mic_pos_m[j][0]) * ux +
                      (s_mic_pos_m[i][1] - s_mic_pos_m[j][1]) * uy) *
                    sample_rate / SOUND_SPEED_MPS;
            }
        }
        s_steering_rate = sample_rate;
        for (int k = 1; k <= GCC_FFT_SIZE / 2; ++k) {
            s_frequency_weight[k] = sqrtf(1800.0f * GCC_FFT_SIZE / ((float)k * sample_rate));
        }
        gcc_phat_reset_history();
        s_noise_initialized = false;
    }
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
        for (int k = 0; k <= GCC_FFT_SIZE / 2; ++k) {
            float p = s_spectrum[ch][k].re * s_spectrum[ch][k].re +
                      s_spectrum[ch][k].im * s_spectrum[ch][k].im;
            if (s_noise_learning) {
                float *noise = &s_noise_power[ch][k];
                if (!s_noise_initialized) *noise = p;
                else *noise += (p < *noise ? .02f : .002f) * (p - *noise);
            }
            s_auto_pow_ema[ch][k] = s_spectral_history_valid
                ? AUTO_POW_EMA_ALPHA * p + (1.0f - AUTO_POW_EMA_ALPHA) * s_auto_pow_ema[ch][k]
                : p;
        }
        if (channel_energy[ch] > max_energy) max_energy = channel_energy[ch];
    }
    if (s_noise_learning) s_noise_initialized = true;
    if (max_energy < 1e-8) {
        gcc_phat_reset_history();
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
        gcc_phat_reset_history();
        return false;
    }
    if (s_active_mic_mask && s_active_mic_mask != out->active_mic_mask) {
        /* A failed/recovered microphone changes the spatial aperture. Do not
         * mix its old pair phases or profile into the degraded estimate. */
        s_score_ema_valid = false;
        memset(s_cross_ema, 0, sizeof(s_cross_ema));
    }
    s_active_mic_mask = out->active_mic_mask;

    memset(s_instant_scores, 0, sizeof(s_instant_scores));
    float instant_weight_sum = 0.0f;
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
        memset(s_work, 0, sizeof(s_work));
        for (int k = 1; k < GCC_FFT_SIZE / 2; ++k) {
            float bin_hz = (float)k * sample_rate / GCC_FFT_SIZE;
            if (bin_hz < SRP_MIN_FREQ_HZ || bin_hz > SRP_MAX_FREQ_HZ) {
                s_work[k].re = 0.0f;
                s_work[k].im = 0.0f;
                continue;
            }
            float re = a[k].re * b[k].re + a[k].im * b[k].im;
            float im = a[k].im * b[k].re - a[k].re * b[k].im;
            /* Coherence requires averaging the COMPLEX cross spectrum before
             * taking its magnitude. |Xi*conj(Xj)| alone contains no phase
             * consistency information, even for independent noise channels. */
            cpx_t *cross = &s_cross_ema[p][k];
            if (s_spectral_history_valid) {
                cross->re += AUTO_POW_EMA_ALPHA * (re - cross->re);
                cross->im += AUTO_POW_EMA_ALPHA * (im - cross->im);
            } else {
                cross->re = re;
                cross->im = im;
            }
            if (!s_instant_enabled) continue;
            float magnitude = sqrtf(re * re + im * im);
            float gi = s_auto_pow_ema[s_pairs[p][0]][k];
            float gj = s_auto_pow_ema[s_pairs[p][1]][k];
            float coherence_sq = fminf(1.0f,
                (cross->re * cross->re + cross->im * cross->im) / (gi * gj + 1e-18f));
            float pi = a[k].re * a[k].re + a[k].im * a[k].im;
            float pj = b[k].re * b[k].re + b[k].im * b[k].im;
            /* Retain unaliased low-frequency time-delay cues for speech and
             * barks. Quiet spectral bins must not be whitened into equal votes. */
            float weight = coherence_sq * foreground_weight(p, k, pi, pj) *
                           s_frequency_weight[k];
            float inv = magnitude > 1e-9f ? 1.0f / magnitude : 0.0f;
            s_work[k].re = re * inv * weight;
            s_work[k].im = im * inv * weight;
            s_work[GCC_FFT_SIZE - k].re = s_work[k].re;
            s_work[GCC_FFT_SIZE - k].im = -s_work[k].im;
        }
        if (s_instant_enabled) {
            fft_inverse(s_work);
            for (int i = 0; i < GCC_FFT_SIZE; ++i) s_corr[p][i] = s_work[i].re;
            /* 质量权重 × 基线长度权重: 对角对 (80mm) 的 TDOA-角度斜率是相邻对 (56.6mm)
             * 的 1.41 倍, 方向分辨能力更强, 应在 SRP 求和中占更大比重 */
            float instant_weight = pair_peak_quality(
                s_corr[p], pair_max_delay_samples((uint8_t)p, sample_rate)) *
                (pair_spacing_m((uint8_t)p) / (MIC_ARRAY_RADIUS_MM * 2.0f / 1000.0f));
            if (instant_weight >= PAIR_MIN_QUALITY) {
                instant_weight_sum += instant_weight;
                for (int bin = 0; bin < SRP_BIN_COUNT; ++bin) {
                    s_instant_scores[bin] += instant_weight *
                        correlation_at(s_corr[p], s_steering_delay[bin][p]);
                }
            }
        }
        /* Sustained localization uses phase-consistent COMPLEX integration,
         * rather than smoothing angles from competing room reflections.
         * The current-frame profile above remains available for onsets. */
        memset(s_work, 0, sizeof(s_work));
        for (int k = 1; k < GCC_FFT_SIZE / 2; ++k) {
            float hz = (float)k * sample_rate / GCC_FFT_SIZE;
            if (hz < SRP_MIN_FREQ_HZ || hz > SRP_MAX_FREQ_HZ) continue;
            cpx_t cross = s_cross_ema[p][k];
            float magnitude = sqrtf(cross.re * cross.re + cross.im * cross.im);
            float gi = s_auto_pow_ema[s_pairs[p][0]][k];
            float gj = s_auto_pow_ema[s_pairs[p][1]][k];
            float coherence_sq = fminf(1.0f,
                (cross.re * cross.re + cross.im * cross.im) / (gi * gj + 1e-18f));
            float inv = magnitude > 1e-9f
                ? coherence_sq * foreground_weight(p, k, gi, gj) *
                  s_frequency_weight[k] / magnitude : 0.0f;
            s_work[k].re = cross.re * inv;
            s_work[k].im = cross.im * inv;
            s_work[GCC_FFT_SIZE-k].re = s_work[k].re;
            s_work[GCC_FFT_SIZE-k].im = -s_work[k].im;
        }
        fft_inverse(s_work);
        for (int i = 0; i < GCC_FFT_SIZE; ++i) s_corr[p][i] = s_work[i].re;
        s_pair_weight[p] = pair_peak_quality(
            s_corr[p], pair_max_delay_samples((uint8_t)p, sample_rate)) *
            (pair_spacing_m((uint8_t)p) / (MIC_ARRAY_RADIUS_MM * 2.0f / 1000.0f));
    }

    s_spectral_history_valid = true;
    float zero_sum = 0.0f, zero_weights = 0.0f;
    for (int p = 0; p < PAIR_COUNT; ++p) {
        if (s_pair_weight[p] < PAIR_MIN_QUALITY) continue;
        zero_sum += s_pair_weight[p] * s_corr[p][0];
        zero_weights += s_pair_weight[p];
    }
    float zero_delay_score = zero_weights > 0.0f ? zero_sum / zero_weights : 0.0f;
    for (int bin = 0; bin < SRP_BIN_COUNT; ++bin) {
        float score = 0.0f;
        float weight_sum = 0.0f;
        for (int p = 0; p < PAIR_COUNT; ++p) {
            uint8_t pair_mask = (uint8_t)((1U << s_pairs[p][0]) | (1U << s_pairs[p][1]));
            if ((out->active_mic_mask & pair_mask) != pair_mask) continue;
            if (s_pair_weight[p] < PAIR_MIN_QUALITY) continue;
            score += s_pair_weight[p] * correlation_at(s_corr[p], s_steering_delay[bin][p]);
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
    find_best_bin(s_scores, &raw_best);
    if (!isfinite(raw_best)) {
        s_score_ema_valid = false;
        return false;
    }
    float instant_second;
    if (instant_weight_sum > 0) {
        for (int bin = 0; bin < SRP_BIN_COUNT; ++bin) s_instant_scores[bin] /= instant_weight_sum;
    }
    if (s_instant_enabled) {
        measure_peak(s_instant_scores, &out->instant_angle_deg, &out->instant_score,
                     &instant_second, &out->instant_confidence, &out->instant_mirror_ratio);
    }
    if (!s_score_ema_valid) {
        memcpy(s_score_ema, s_scores, sizeof(s_scores));
        s_zero_delay_ema = zero_delay_score;
        s_score_ema_valid = true;
    } else {
        s_zero_delay_ema = s_score_ema_alpha * zero_delay_score +
                          (1.0f - s_score_ema_alpha) * s_zero_delay_ema;
        for (int bin = 0; bin < SRP_BIN_COUNT; ++bin) {
            s_score_ema[bin] = s_score_ema_alpha * s_scores[bin] +
                (1.0f - s_score_ema_alpha) * s_score_ema[bin];
        }
    }

    float angle, best, second, confidence, mirror_ratio;
    measure_peak(s_score_ema, &angle, &best, &second, &confidence, &mirror_ratio);

    out->angle_deg = angle;
    out->score = best;
    out->zero_delay_score = s_zero_delay_ema;
    out->second_score = second;
    out->confidence = confidence;
    out->mirror_ratio = mirror_ratio;
    out->valid = isfinite(angle) && isfinite(confidence) && best > 0.0f &&
                 confidence >= SRP_MIN_CONFIDENCE;
    collect_display_peaks(mic, count, sample_rate, out);
    return out->valid;
}
