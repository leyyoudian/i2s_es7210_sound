/*
 * 总合演示: 鸟声检测 + 声源定位
 *
 * 硬件: ESP32-S3 + ES7210 + 4x SPH1642HT5H-1 圆形麦克风阵列 (40mm 孔径)
 * 功能:
 *   1. 4通道 I2S TDM 音频同步采集 (48kHz)
 *   2. GCC-PHAT + SRP 声源定位 (方位角估计, 48kHz 全速率处理)
 *   3. Edge Impulse 鸟声/噪声分类 (TFLite Micro, INT8, 16kHz 输入经 3:1 抗混叠抽取)
 *   4. 16 颗 WS2812 LED 方向指示 + Wi-Fi Dashboard 实时显示
 *
 * 如需运行单个外设例程 (I2C/ADC/UART/RGB/ES7210/USB),
 * 见 examples/examples.h 中的 RUN_EXAMPLE_xxx 开关.
 */

#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/i2s_tdm.h"
#include "driver/i2c.h"
#include "es7210.h"

#include "doa/gcc_phat.h"
#include "doa/doa_display_gate.h"
#include "doa/doa_event.h"
#include "doa/doa_tracker.h"
#include "doa/direction_mapping.h"
#include "bird_ai/bird_decision.h"
#include "bird_ai/ei_inference.h"
#include "model-parameters/model_metadata.h"
#include "wifi/wifi_server.h"
#include "led/led_indicator.h"
#include "board_test/board_test.h"
#include "app_mode.h"
#include "examples/examples.h"

static const char *TAG = "bird_det";

static void run_bird_doa_demo(void);

/* ================================================================ */
/*  硬件引脚定义                                                     */
/* ================================================================ */

/* I2C (ES7210 寄存器控制) */
#define I2C_NUM             (0)
#define I2C_SDA_IO          (8)
#define I2C_SCL_IO          (9)

/* I2S (ES7210 音频数据) */
#define I2S_MCK_IO          (16)    /* MCLK */
#define I2S_BCK_IO          (18)    /* SCLK / BCLK */
#define I2S_WS_IO           (17)    /* LRCK / WS */
#define I2S_DI_IO           (15)    /* Serial Data In */

/* ================================================================ */
/*  音频参数                                                         */
/* ================================================================ */

/* 48kHz 采集: r=25mm 阵列最大 TDOA 仅 50mm/c ≈ 146µs,
 * 16kHz 下只有 ±2.3 样本, 1 个采样误差 ≈ 25° 角度误差;
 * 48kHz 使 TDOA 分辨率 ×3, 并允许 PHAT 频带扩展到 9kHz.
 * EI 模型仍为 16kHz 输入, 经 3:1 抗混叠抽取得到. */
#define SAMPLE_RATE         (48000)
#define EI_SAMPLE_RATE      (16000)     /* EI 模型输入采样率 (抽取后) */
#define SAMPLE_BITS         (I2S_DATA_BIT_WIDTH_16BIT)
#define CHAN_NUM             (4)
#define MCLK_MULTIPLE       (I2S_MCLK_MULTIPLE_256)
#define TDM_SLOT_MASK       (I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3)
#define I2S_FMT             (ES7210_I2S_FMT_I2S)

/* ES7210 编解码器设置 */
#define ES7210_ADDR         (0x40)
#define ES7210_ADDR_MIN     (0x40)
#define ES7210_ADDR_MAX     (0x43)
#define ES7210_I2C_CLK_HZ   (100000)
#define ES7210_GAIN         (ES7210_MIC_GAIN_30DB)  /* 30dB 增益, 适合户外拾音 */
#define ES7210_BIAS         (ES7210_MIC_BIAS_2V87)
/* 数字音量 +12dB: 默认 0dB 时环境底噪仅 ~-66dBFS, 距 16bit 量化底 (-90dBFS)
 * 只有一点点余量, 弱信号 (远距/小声) 的有效 SNR 被量化噪声吃掉
 * (人耳听得见而板子听不见的根源). board_test 录音同样用了 +24dB.
 * 12dB 而非 24dB 是为近场大声源 (响指/拍手) 保留峰值余量防数字削顶;
 * 电平整体抬升后各 SNR 自适应门限会跟随噪声底, 不需要改. */
#define ES7210_VOL          (12)

/* ================================================================ */
/*  处理参数                                                         */
/* ================================================================ */

/* GCC-PHAT 分析帧长 (对应 GCC_FFT_SIZE): 1024 点 @48kHz = 21.3ms.
 * 帧率 46.9fps, 是原 16kHz 配置的 3 倍, 下列按帧更新的平滑系数均已换算. */
#define FRAME_LEN           (1024)

/* EI 模型单次推理所需采样数 */
#define EI_SAMPLES          (EI_CLASSIFIER_RAW_SAMPLE_COUNT)

/* I2S 每次读取的 int16 样本数 = FRAME_LEN * 4ch */
#define I2S_BUF_SAMPLES     (FRAME_LEN * CHAN_NUM)

/* 分类稳定化参数: 抑制噪音误判为鸟叫 */
#define CLS_SMOOTH_ALPHA    (0.55f)
#define EI_WINDOW_MIN_RMS   (0.0015f)

/* EI 输入增益控制: 提升弱鸟叫召回, 并限制过放大 */
#define EI_TARGET_RMS       (0.045f)
#define EI_RMS_MIN_FOR_AGC  (0.0012f)
#define EI_GAIN_MIN         (0.70f)
#define EI_GAIN_MAX         (6.00f)
/* 0.09 @21.3ms 帧 ≈ 原 0.25 @64ms 帧的时间常数 (~200ms) */
#define EI_GAIN_SMOOTH_ALPHA (0.09f)

/* 声源有效性门限: 频带 SNR 自适应门控.
 * 门控能量用 GCC 带内 (1.5~6kHz) rms 而非宽带 rms: 目标声源 (鸟叫/啁啾/
 * 瞬态) 能量集中在带内, 而环境噪声 (风扇/电源/低频) 大部分在带外, 带内
 * SNR 天然高于宽带 SNR, 远距/小声源可提前过门限.
 * 触发门限 = max(绝对下限, 噪声底×倍率): 安静环境自动变灵敏, 嘈杂环境
 * 自动收紧. 噪声底仅在未激活且本帧能量不显著高于底时更新. */
#define RMS_FLOOR_MIN           (0.00015f)
/* 噪声底上升要慢 (~2.6s @46.9fps): 持续性小声源 (音乐/说话) 若等于或略高于
 * 门限, 快速跟踪会把它学进噪声底导致门限追涨、永远不触发 (噪声门经典缺陷).
 * 回落要快 (~0.35s): 噪声环境变安静后门限及时下降恢复灵敏度. */
#define RMS_FLOOR_EMA_ALPHA     (0.008f)
#define RMS_FLOOR_DECAY_ALPHA   (0.06f)
#define LED_SNR_STRONG_RATIO    (2.4f)
#define LED_SNR_STRONG_ABS_MIN  (0.0006f)
#define LED_SNR_WEAK_RATIO      (1.8f)
#define LED_SNR_WEAK_ABS_MIN    (0.0003f)
#define LED_SNR_OFF_RATIO       (1.35f)
#define LED_SNR_OFF_ABS_MIN     (0.00025f)
#define LED_WEAK_SRP_CONF_THRESHOLD (0.05f)
/* 噪声底上升极慢 (~27s @46.9fps): 适应环境变吵; 下降快 (见更新逻辑).
 * 组合即"最小统计量"噪声估计: 任何声音的间隙都会暴露真实底噪,
 * floor 自动落到最低处; 瞬态尖峰来不及抬升 floor. */
#define RMS_FLOOR_RISE_ALPHA    (0.0008f)

/* 跟踪有效性 (localization_valid) 的能量门限: 同样改为带内 SNR 自适应 */
#define DOA_TRACK_MIN_ENERGY_ABS    (0.0004f)
#define DOA_TRACK_MIN_ENERGY_RATIO  (1.6f)

/* 低声压下 SRP 容易出现反射/对面假峰, 只用于点亮保持, 不用于更新方向.
 * 注: score 值会随频带宽度变化; 恢复 1500~6000Hz 宽带后 score 降到 ~0.01,
 * 故阈值从 0.040 同步下调到 0.010. */
#define DOA_TRACK_MIN_CONF          (0.025f)
#define DOA_TRACK_MIN_SCORE         (0.010f)

/* 小信号时四路 RMS 很接近, 频繁切 EI 输入通道会让模型输入音色抖动 */
#define EI_CHANNEL_SWITCH_RATIO     (1.35f)

/* Short transients such as finger snaps are diluted by a 2048-sample RMS frame. */
#define TRANSIENT_BLOCK_SAMPLES     (384)   /* 8ms @48kHz */
/* 瞬态激活门限: 0.008 而非 0.0045 —— 持续音乐/说话的 8ms 峰值块会骑在
 * 0.0045 上导致 act 反复触发->噪声底永不更新->弱源门限被冻在高位.
 * 响指/敲击类瞬态通常 >= 0.01, 不受影响. */
#define TRANSIENT_RMS_ON_THRESHOLD  (0.0080f)
#define TRANSIENT_DOA_CONF_BOOST    (0.36f)

/* 镜像含糊门限: SRP 对侧 (±180°) 分数达到主峰该比例时, 本帧方向视为含糊,
 * 瞬态 (响指) 不走 1~2 帧快速通道, 退回连续帧确认, 避免响指亮到对面.
 * 十字阵列中心对称, 对径麦 80mm 在 >2.14kHz 空间混叠, 响指/拍手能量
 * 恰好集中在该混叠频段, 是对侧误亮的主要来源. */
#define TRANSIENT_MIRROR_RATIO_MAX  (0.60f)

/* RMS 平滑系数: 0.11 @21.3ms 帧 ≈ 原 0.30 @64ms 帧的时间常数 */
#define RMS_SMOOTH_ALPHA    (0.11f)

/* GCC-PHAT 预滤波 (Fs=48k): 覆盖 SRP 频带 1500~6000Hz.
 * HP 约 1.5kHz + LP 约 6kHz. 一阶 IIR 系数: HP a=RC/(RC+dt), LP a=dt/(dt+RC).
 * 注意: 系数与采样率绑定! 16kHz 时代的 0.629/0.702 搬到 48kHz 会把滤波带
 * 漂移到 4.5~18kHz, SRP 实际只剩 4.5~6kHz 一小段 (处理增益 -6dB ≈ 距离减半,
 * 且全部落在 80mm 对径基线 2.14kHz 空间混叠区之上, 镜像伪影加重).
 * 48k 换算: HP RC=1/(2π·1.5k)=106.1µs -> a=RC/(RC+20.83µs)=0.836;
 *           LP RC=1/(2π·6k)=26.53µs  -> a=20.83µs/(RC+dt)=0.440. */
#define GCC_HP_ALPHA                (0.836f)
#define GCC_LP_ALPHA                (0.440f)

/* DOA 输出门控: 仅在鸟叫可信时更新方向 */
#define DOA_BIRD_CONF_GATE          (0.40f)

/* 网页角度推送节奏 (ms) */
#define UI_PUSH_PERIOD_MS   (100)

/* 任务参数 */
#define AUDIO_TASK_STACK    (8192)
#define AUDIO_TASK_PRIO     (5)
#define INFERENCE_TASK_STACK (8192)
#define INFERENCE_TASK_PRIO  (3)
#define EI_BUFFER_COUNT      (1)

/* ================================================================ */
/*  全局状态                                                         */
/* ================================================================ */

static SemaphoreHandle_t s_result_mtx = NULL;
static QueueHandle_t s_ei_ready_queue = NULL;
static QueueHandle_t s_ei_free_queue = NULL;
static float s_doa_angle       = 0.0f;
static float s_bird_conf       = 0.0f;
static float s_noise_conf      = 0.0f;
static bool  s_bird_detected   = false;
static doa_display_gate_t s_doa_display_gate;

/* 音频缓冲区 */
static int16_t s_i2s_buf[I2S_BUF_SAMPLES];           /* I2S TDM 交织原始数据 */
static int16_t s_ch[CHAN_NUM][FRAME_LEN];             /* 解交织后各通道数据 */
#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
static int16_t s_ch_gcc[CHAN_NUM][FRAME_LEN];         /* GCC 预滤波输入 */
#endif
#if APP_MODE == APP_MODE_SOUND_CLASSIFY
static int16_t s_ei_buf[EI_BUFFER_COUNT][EI_SAMPLES]; /* EI 推理双缓冲 */
#endif
static int     s_ei_pos = 0;                           /* 当前写入位置 */
static int     s_ei_fill_index = -1;
static float   s_rms_smooth = 0.0f;
static bool    s_rms_valid = false;
static float   s_rms_floor = 0.0010f;   /* 环境噪声底跟踪 (自适应门限基准) */
static bool    s_source_active = false;
static unsigned s_sustained_frames = 0; /* 连续有声且非瞬态的帧数 (持续模式判定) */
/* LED 熄灭保持: 显示失活后须持续这么久才真正熄灯, 防止门限边缘
 * 1 秒内亮灭数次 (抽搐式闪烁). 亮起无延迟. */
#define LED_OFF_HOLD_FRAMES  45U
static unsigned s_led_hold_frames = 0;
static bool     s_led_lit = false;
static float   s_last_bird_conf = 0.0f;
static float   s_last_noise_conf = 1.0f;
static bool    s_last_bird_detected = false;
static bool    s_cls_valid = false;
static float   s_bird_conf_smooth = 0.0f;
static float   s_noise_conf_smooth = 1.0f;
static int     s_ei_src_ch = 0;
static bool    s_agc_valid = false;
static float   s_ei_gain = 1.0f;
static double  s_ei_energy_sum = 0.0;
static uint32_t s_ei_energy_count = 0;
static doa_tracker_t s_doa_tracker;
static bird_decision_t s_bird_decision;

typedef struct {
    uint8_t buffer_index;
    uint8_t source_channel;
    float window_rms;
    float gain;
} inference_job_t;

typedef struct {
    float x_prev;
    float hp_prev;
    float lp_prev;
    bool initialized;
} gcc_filter_state_t;

static gcc_filter_state_t s_gcc_filter[CHAN_NUM];

static i2s_chan_handle_t s_i2s_rx = NULL;
static es7210_dev_handle_t s_es7210 = NULL;

/* ================================================================ */
/*  I2S TDM 初始化                                                   */
/* ================================================================ */
static void init_i2s(void)
{
    ESP_LOGI(TAG, "I2S TDM 初始化 (%d Hz, %d ch)", SAMPLE_RATE, CHAN_NUM);

    i2s_chan_config_t rx_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_cfg, NULL, &s_i2s_rx));

    i2s_tdm_config_t tdm_cfg = {
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
                        SAMPLE_BITS, I2S_SLOT_MODE_STEREO, TDM_SLOT_MASK),
        .clk_cfg = {
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .sample_rate_hz = SAMPLE_RATE,
            .mclk_multiple = MCLK_MULTIPLE
        },
        .gpio_cfg = {
            .mclk = I2S_MCK_IO,
            .bclk = I2S_BCK_IO,
            .ws   = I2S_WS_IO,
            .dout = -1,           /* ES7210 仅有 ADC, 无 DAC */
            .din  = I2S_DI_IO
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(s_i2s_rx, &tdm_cfg));
}

/* ================================================================ */
/*  ES7210 编解码器初始化 (I2C 控制)                                  */
/* ================================================================ */
static esp_err_t i2c_probe_addr(i2c_port_t port, uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = ESP_OK;
    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    ret |= i2c_master_stop(cmd);
    if (ret == ESP_OK) {
        ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(30));
    }

    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t init_es7210(void)
{
    ESP_LOGI(TAG, "ES7210 编解码器配置");

    i2c_config_t i2c_cfg = {
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = ES7210_I2C_CLK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));

    uint8_t detected_addr = 0;
    bool found = false;

    for (uint8_t addr = ES7210_ADDR_MIN; addr <= ES7210_ADDR_MAX; addr++) {
        if (i2c_probe_addr(I2C_NUM, addr) == ESP_OK) {
            ESP_LOGI(TAG, "I2C 发现 ES7210 候选地址: 0x%02X", addr);
            if (!found || addr == ES7210_ADDR) {
                detected_addr = addr;
                found = true;
            }
        }
    }

    if (!found) {
        ESP_LOGE(TAG, "未扫描到 ES7210 (0x40~0x43), 请检查 SDA/SCL/ADDR 引脚与供电");
        return ESP_FAIL;
    }

    if (detected_addr != ES7210_ADDR) {
        ESP_LOGW(TAG, "ES7210 地址不是默认 0x%02X, 将使用 0x%02X", ES7210_ADDR, detected_addr);
    }

    es7210_i2c_config_t es_i2c = {
        .i2c_port = I2C_NUM,
        .i2c_addr = detected_addr
    };

    ESP_RETURN_ON_ERROR(es7210_new_codec(&es_i2c, &s_es7210), TAG, "创建 ES7210 句柄失败");

    es7210_codec_config_t codec_cfg = {
        .i2s_format = I2S_FMT,
        .mclk_ratio = MCLK_MULTIPLE,
        .sample_rate_hz = SAMPLE_RATE,
        .bit_width = (es7210_i2s_bits_t)SAMPLE_BITS,
        .mic_bias = ES7210_BIAS,
        .mic_gain = ES7210_GAIN,
        .flags.tdm_enable = true
    };

    esp_err_t ret = ESP_FAIL;
    for (int retry = 1; retry <= 3; retry++) {
        ret = es7210_config_codec(s_es7210, &codec_cfg);
        if (ret == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "ES7210 配置失败(第 %d/3 次): %s", retry, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (ret != ESP_OK) {
        es7210_del_codec(s_es7210);
        s_es7210 = NULL;
        return ret;
    }

    ESP_RETURN_ON_ERROR(es7210_config_volume(s_es7210, ES7210_VOL), TAG, "设置 ES7210 音量失败");
    return ESP_OK;
}

/* ================================================================ */
/*  TDM 数据解交织: [S0,S1,S2,S3,S0,...] -> ch0[],ch1[],ch2[],ch3[] */
/* ================================================================ */
static void deinterleave(const int16_t *src, int frames)
{
    for (int i = 0; i < frames; i++) {
        s_ch[0][i] = src[i * CHAN_NUM + 0];   /* MIC1 (270 deg) */
        s_ch[1][i] = src[i * CHAN_NUM + 1];   /* MIC2 (180 deg) */
        s_ch[2][i] = src[i * CHAN_NUM + 2];   /* MIC3 (0 deg)   */
        s_ch[3][i] = src[i * CHAN_NUM + 3];   /* MIC4 (90 deg)  */
    }
}

static void bandpass_for_gcc_i16(const int16_t *src, int16_t *dst, int n,
                                 gcc_filter_state_t *state)
{
    if (n <= 0 || state == NULL) {
        return;
    }

    int start = 0;
    if (!state->initialized) {
        state->x_prev = (float)src[0];
        state->hp_prev = 0.0f;
        state->lp_prev = 0.0f;
        state->initialized = true;
        dst[0] = 0;
        start = 1;
    }

    for (int i = start; i < n; i++) {
        float x = (float)src[i];
        float hp = GCC_HP_ALPHA * (state->hp_prev + x - state->x_prev);
        float lp = state->lp_prev + GCC_LP_ALPHA * (hp - state->lp_prev);

        state->hp_prev = hp;
        state->lp_prev = lp;
        state->x_prev = x;

        if (lp > 32767.0f) {
            lp = 32767.0f;
        } else if (lp < -32768.0f) {
            lp = -32768.0f;
        }
        dst[i] = (int16_t)lrintf(lp);
    }
}

static bool doa_gate_is_open(bool bird_detected, float bird_conf)
{
    return bird_detected && (bird_conf >= DOA_BIRD_CONF_GATE);
}

static float calc_rms_norm_i16(const int16_t *x, int n)
{
    if (n <= 0) {
        return 0.0f;
    }

    float sum_sq = 0.0f;
    for (int i = 0; i < n; i++) {
        float v = (float)x[i] / 32768.0f;
        sum_sq += v * v;
    }
    return sqrtf(sum_sq / (float)n);
}

static float calc_peak_block_rms_norm_i16(const int16_t *x, int n, int block)
{
    if (x == NULL || n <= 0 || block <= 0) {
        return 0.0f;
    }

    float peak = 0.0f;
    for (int start = 0; start < n; start += block) {
        int end = start + block;
        if (end > n) {
            end = n;
        }
        float sum_sq = 0.0f;
        for (int i = start; i < end; ++i) {
            float v = (float)x[i] / 32768.0f;
            sum_sq += v * v;
        }
        float block_rms = sqrtf(sum_sq / (float)(end - start));
        if (block_rms > peak) {
            peak = block_rms;
        }
    }
    return peak;
}

/* 两路零延迟归一化互相关 (-1~1). 用于诊断麦克风极性/相位一致性:
 * 若某一路与其它路相关接近 0 或为负, 说明该路极性接反或焊接/通道错位. */
static float zero_lag_corr(const int16_t *a, const int16_t *b, int n)
{
    if (a == NULL || b == NULL || n <= 0) {
        return 0.0f;
    }
    double sa = 0.0, sb = 0.0, sab = 0.0;
    for (int i = 0; i < n; ++i) {
        double x = (double)a[i];
        double y = (double)b[i];
        sa += x * x;
        sb += y * y;
        sab += x * y;
    }
    if (sa <= 0.0 || sb <= 0.0) {
        return 0.0f;
    }
    return (float)(sab / sqrt(sa * sb));
}

/* ---- 近场瞬态 onset 定位 (抗 ADC 削顶) ----
 * 满量程仅对应 ~64dB SPL, 20cm 级响指/拍手在麦克风处可达 95dB+, 必然硬削顶;
 * 削顶波形做互相关/SRP 全是垃圾 (表现为"一个灯都不亮"或角度乱跳), 且
 * 远场平面波假设在 <0.5m 时 TDOA 误差可达 1~2 采样 (~30°).
 * 改用各通道能量累积起点 (onset) 估计 TDOA: 到达先后不受削顶影响;
 * 再用球面波模型 (近场) 在 方位×距离 小网格上拟合方向. */
#define ONSET_PEAK_FRAC     (0.30f)    /* 首次超过峰值该比例视为到达 */
#define ONSET_MAX_RESIDUAL  (2.5f)     /* 拟合残差上限 (采样点) */
#define NF_CLIP_SAMPLES     (2)        /* 削顶样本数阈值比例分母: > n/NF 判削顶 */
#define NF_SNAP_TRANSIENT_MIN (0.0100f) /* 近场通道要求的瞬态强度 */
#define NF_SNAP_CONFIDENCE  (0.65f)    /* 高于对侧镜像门限 0.60, 优质拟合可越对侧 */
static const float s_nf_mic_pos_m[CHAN_NUM][2] = {
    {0.0f, -0.040f}, {-0.040f, 0.0f}, {0.040f, 0.0f}, {0.0f, 0.040f},
};

/* 通道 onset: 首次超过本通道峰值 30% 的时间点 (对削顶/反射均稳健:
 * 削顶时峰值即饱和值, 首次越过 30% 饱和值 = 直达波到达; 桌面等强反射
 * 出现在直达波之后, 不影响首次越过点). 失败返回 -1. */
static float channel_onset_time(const int16_t *x, int n)
{
    int peak = 0;
    for (int i = 0; i < n; ++i) {
        int a = x[i] > 0 ? x[i] : -x[i];
        if (a > peak) {
            peak = a;
        }
    }
    if (peak <= 0) {
        return -1.0f;
    }
    int thresh = peak * 3 / 10;
    for (int i = 0; i < n; ++i) {
        int a = x[i] > 0 ? x[i] : -x[i];
        if (a >= thresh) {
            return (float)i;
        }
    }
    return -1.0f;
}

/* onset TDOA + 球面波网格拟合: 成功返回 true 并给出原始坐标系角度 */
static bool onset_doa_estimate(const int16_t *const ch[CHAN_NUM], int n,
                               float *angle_deg, float *residual)
{
    float onset[CHAN_NUM];
    for (int c = 0; c < CHAN_NUM; ++c) {
        onset[c] = channel_onset_time(ch[c], n);
        if (onset[c] < 0.0f) {
            return false;
        }
    }
    static const float dists_m[] = {0.15f, 0.25f, 0.45f, 2.0f};
    float best_err = 1e30f;
    float best_angle = 0.0f;
    for (int id = 0; id < (int)(sizeof(dists_m) / sizeof(dists_m[0])); ++id) {
        float r = dists_m[id];
        for (int bin = 0; bin < 180; ++bin) {
            float theta = (float)bin * 2.0f * (float)M_PI / 180.0f;
            float sx = r * cosf(theta), sy = r * sinf(theta);
            float path[CHAN_NUM];
            for (int c = 0; c < CHAN_NUM; ++c) {
                float dx = s_nf_mic_pos_m[c][0] - sx;
                float dy = s_nf_mic_pos_m[c][1] - sy;
                path[c] = sqrtf(dx * dx + dy * dy);
            }
            float err = 0.0f;
            for (int i = 0; i < CHAN_NUM; ++i) {
                for (int j = i + 1; j < CHAN_NUM; ++j) {
                    float model = (path[i] - path[j]) / SOUND_SPEED_MPS *
                                  (float)SAMPLE_RATE;
                    float d = onset[i] - onset[j] - model;
                    err += d * d;
                }
            }
            if (err < best_err) {
                best_err = err;
                best_angle = (float)bin * 2.0f;
            }
        }
    }
    *residual = sqrtf(best_err / 6.0f);
    *angle_deg = best_angle;
    return *residual <= ONSET_MAX_RESIDUAL;
}

static int count_clipped(const int16_t *x, int n)
{
    int count = 0;
    for (int i = 0; i < n; ++i) {
        if (x[i] >= 32000 || x[i] <= -32000) {
            count++;
        }
    }
    return count;
}


static int choose_ei_channel(const float *rms_ch, int n, int current)
{
    int best_idx = 0;
    float best_val = (n > 0) ? rms_ch[0] : 0.0f;
    for (int i = 1; i < n; i++) {
        if (rms_ch[i] > best_val) {
            best_val = rms_ch[i];
            best_idx = i;
        }
    }
    if (current >= 0 && current < n &&
        rms_ch[current] > 0.0f &&
        best_val < rms_ch[current] * EI_CHANNEL_SWITCH_RATIO) {
        return current;
    }
    return best_idx;
}

/* ---- 48k -> 16k 抗混叠抽取 (EI 模型输入链路) ----
 * 12 阶 Hamming 窗 sinc FIR, fc≈8kHz @48kHz, 系数已归一化 (Σh=1).
 * 每消费 3 个 48kHz 输入样本产出 1 个 16kHz 样本. */
#if SAMPLE_RATE % EI_SAMPLE_RATE != 0
#error "EI_SAMPLE_RATE 必须整除 SAMPLE_RATE"
#endif
#define EI_DECIM_FACTOR     (SAMPLE_RATE / EI_SAMPLE_RATE)
#define EI_DECIM_TAPS       (12)
static const float s_ei_decim_taps[EI_DECIM_TAPS] = {
    -0.0023130f, -0.0108148f, -0.0158521f, 0.0385116f, 0.1783608f, 0.3121075f,
    0.3121075f, 0.1783608f, 0.0385116f, -0.0158521f, -0.0108148f, -0.0023130f,
};

typedef struct {
    float hist[EI_DECIM_TAPS];  /* hist[EI_DECIM_TAPS-1] 为最新样本 */
    uint32_t phase;             /* 窗口内已消费的输入样本计数 */
} ei_decim_state_t;

static ei_decim_state_t s_ei_decim;

static void ei_decim_reset(void)
{
    memset(&s_ei_decim, 0, sizeof(s_ei_decim));
}

/* 抽取并应用增益后写入 dst, 返回写入的 16kHz 样本数 */
static int ei_decim_push(int16_t *dst, int dst_cap, const int16_t *src, int n,
                         float gain)
{
    int written = 0;
    for (int i = 0; i < n && written < dst_cap; i++) {
        for (int k = 0; k < EI_DECIM_TAPS - 1; ++k) {
            s_ei_decim.hist[k] = s_ei_decim.hist[k + 1];
        }
        s_ei_decim.hist[EI_DECIM_TAPS - 1] = (float)src[i];
        s_ei_decim.phase++;
        if ((s_ei_decim.phase % EI_DECIM_FACTOR) != 0U) {
            continue;
        }
        float acc = 0.0f;
        for (int k = 0; k < EI_DECIM_TAPS; ++k) {
            acc += s_ei_decim_taps[k] * s_ei_decim.hist[k];
        }
        float scaled = acc * gain;
        if (scaled > 32767.0f) {
            scaled = 32767.0f;
        } else if (scaled < -32768.0f) {
            scaled = -32768.0f;
        }
        dst[written++] = (int16_t)lrintf(scaled);
    }
    return written;
}

static bool update_source_active(float brms_raw, float transient_rms,
                                 bool srp_valid, float srp_confidence)
{
    /* 噪声底跟踪 (最小统计量式): 下降快 (~0.2s) 跟随任何更低能量,
     * 上升极慢 (~27s) 适应环境变吵. 持续声源 (音乐) 的音符间隙会暴露
     * 真实底噪使 floor 落到最低处, 门限不会骑到声源上 (死锁防护) */
    if (brms_raw < s_rms_floor) {
        s_rms_floor += RMS_FLOOR_DECAY_ALPHA * (brms_raw - s_rms_floor);
    } else {
        s_rms_floor += RMS_FLOOR_RISE_ALPHA * (brms_raw - s_rms_floor);
    }
    if (s_rms_floor < RMS_FLOOR_MIN) {
        s_rms_floor = RMS_FLOOR_MIN;
    }

    if (!s_rms_valid) {
        s_rms_smooth = brms_raw;
        s_rms_valid = true;
    } else {
        s_rms_smooth = (RMS_SMOOTH_ALPHA * brms_raw) +
                       ((1.0f - RMS_SMOOTH_ALPHA) * s_rms_smooth);
    }

    float strong_gate = fmaxf(LED_SNR_STRONG_ABS_MIN,
                              s_rms_floor * LED_SNR_STRONG_RATIO);
    float weak_gate = fmaxf(LED_SNR_WEAK_ABS_MIN,
                            s_rms_floor * LED_SNR_WEAK_RATIO);
    float off_gate = fmaxf(LED_SNR_OFF_ABS_MIN,
                           s_rms_floor * LED_SNR_OFF_RATIO);
    float transient_gate = fmaxf(TRANSIENT_RMS_ON_THRESHOLD,
                                 s_rms_floor * 3.5f);

    bool strong_source = s_rms_smooth >= strong_gate ||
                         transient_rms >= transient_gate;
    bool coherent_weak_source = s_rms_smooth >= weak_gate &&
                                srp_valid &&
                                srp_confidence >= LED_WEAK_SRP_CONF_THRESHOLD;

    if (!s_source_active && (strong_source || coherent_weak_source)) {
        s_source_active = true;
    } else if (s_source_active && s_rms_smooth <= off_gate) {
        s_source_active = false;
    }

    return s_source_active;
}

#if APP_MODE == APP_MODE_SOUND_CLASSIFY

static void inference_task(void *arg)
{
    inference_job_t job;
    ESP_LOGI(TAG, "Inference task started");

    while (xQueueReceive(s_ei_ready_queue, &job, portMAX_DELAY) == pdTRUE) {
        float bird_c = 0.0f;
        float noise_c = 1.0f;
        bool classified = false;
        bool silence_gated = false;

        if (job.window_rms < EI_WINDOW_MIN_RMS) {
            classified = true;
            silence_gated = true;
        } else {
            ei_inference_result_t res = {0};
            if (ei_run_inference(s_ei_buf[job.buffer_index], EI_SAMPLES, &res) == 0) {
                bird_c = res.bird_confidence;
                noise_c = res.noise_confidence;
                classified = true;
            }
        }

        if (classified) {
            if (!s_cls_valid) {
                s_bird_conf_smooth = bird_c;
                s_noise_conf_smooth = noise_c;
                s_cls_valid = true;
            } else {
                s_bird_conf_smooth = CLS_SMOOTH_ALPHA * bird_c +
                    (1.0f - CLS_SMOOTH_ALPHA) * s_bird_conf_smooth;
                s_noise_conf_smooth = CLS_SMOOTH_ALPHA * noise_c +
                    (1.0f - CLS_SMOOTH_ALPHA) * s_noise_conf_smooth;
            }

            s_last_bird_detected = bird_decision_update(
                &s_bird_decision, bird_c, noise_c,
                s_bird_conf_smooth, s_noise_conf_smooth);
            s_last_bird_conf = s_bird_conf_smooth;
            s_last_noise_conf = s_noise_conf_smooth;

            float angle_snapshot = 0.0f;
            if (xSemaphoreTake(s_result_mtx, pdMS_TO_TICKS(10)) == pdTRUE) {
                s_bird_conf = s_last_bird_conf;
                s_noise_conf = s_last_noise_conf;
                s_bird_detected = s_last_bird_detected;
                angle_snapshot = s_doa_angle;
                xSemaphoreGive(s_result_mtx);
            }

            ESP_LOGI(TAG,
                     "DOA=%.1f src=async ei_ch=%u window_rms=%.4f gain=%.2f "
                     "gate=%s bird=%.1f%% noise=%.1f%% state=%s",
                     angle_snapshot, job.source_channel, job.window_rms, job.gain,
                     silence_gated ? "silence" : "model",
                     bird_c * 100.0f, noise_c * 100.0f,
                     s_last_bird_detected ? "bird" : "noise");
        }

        uint8_t free_index = job.buffer_index;
        xQueueSend(s_ei_free_queue, &free_index, portMAX_DELAY);

        /* The model window arrives about as fast as MFCC inference completes.
         * Block briefly so CPU0's idle task can service the task watchdog. */
        vTaskDelay(1);
    }
}
#endif /* APP_MODE == APP_MODE_SOUND_CLASSIFY */

/* ================================================================ */
/*  音频处理任务 - 采集 / 定位 / 推理 循环                            */
/* ================================================================ */
static void audio_task(void *arg)
{
    ESP_LOGI(TAG, "音频处理任务启动");
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_rx));
    uint32_t frame_idx = 0;
    TickType_t last_ui_push_tick = 0;

    while (1) {
        size_t bytes_read = 0;

        /* 读取一帧 4 通道 TDM 数据 */
        esp_err_t ret = i2s_channel_read(
            s_i2s_rx, s_i2s_buf, sizeof(s_i2s_buf),
            &bytes_read, pdMS_TO_TICKS(500));
        if (ret != ESP_OK || bytes_read == 0) {
            continue;
        }

        /* 测量点: 在阻塞读之后, 只统计纯处理时间, 排除读数据阻塞 */
        int64_t frame_start_us = esp_timer_get_time();

        int frames = bytes_read / (CHAN_NUM * sizeof(int16_t));
        deinterleave(s_i2s_buf, frames);

        float rms0 = calc_rms_norm_i16(s_ch[0], frames);
        float rms1 = calc_rms_norm_i16(s_ch[1], frames);
        float rms2 = calc_rms_norm_i16(s_ch[2], frames);
        float rms3 = calc_rms_norm_i16(s_ch[3], frames);
        float rms_ch[CHAN_NUM] = {rms0, rms1, rms2, rms3};
        float rms = fmaxf(fmaxf(rms0, rms1), fmaxf(rms2, rms3));

#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
        /* ---- GCC-PHAT 声源定位 (预滤波后) ---- */
        for (int ch = 0; ch < CHAN_NUM; ch++) {
            bandpass_for_gcc_i16(s_ch[ch], s_ch_gcc[ch], frames, &s_gcc_filter[ch]);
        }
        const int16_t *doa_mics[CHAN_NUM] = {
            s_ch_gcc[0], s_ch_gcc[1], s_ch_gcc[2], s_ch_gcc[3]
        };
        gcc_phat_result_t doa_result = {0};
        bool srp_valid = gcc_phat_estimate_4ch(
            doa_mics, frames, SAMPLE_RATE, &doa_result);
        float angle_raw = direction_calibrate_doa(doa_result.angle_deg);
        float xc01 = zero_lag_corr(s_ch[0], s_ch[1], frames);
        float xc02 = zero_lag_corr(s_ch[0], s_ch[2], frames);
        float xc03 = zero_lag_corr(s_ch[0], s_ch[3], frames);
        float transient0 = calc_peak_block_rms_norm_i16(s_ch[0], frames, TRANSIENT_BLOCK_SAMPLES);
        float transient1 = calc_peak_block_rms_norm_i16(s_ch[1], frames, TRANSIENT_BLOCK_SAMPLES);
        float transient2 = calc_peak_block_rms_norm_i16(s_ch[2], frames, TRANSIENT_BLOCK_SAMPLES);
        float transient3 = calc_peak_block_rms_norm_i16(s_ch[3], frames, TRANSIENT_BLOCK_SAMPLES);
        float transient_rms = fmaxf(fmaxf(transient0, transient1), fmaxf(transient2, transient3));
        /* 带内 (1.5~6kHz) rms: 与 SRP 频带一致, 用于激活/跟踪门控 */
        float brms = fmaxf(fmaxf(calc_rms_norm_i16(s_ch_gcc[0], frames),
                                 calc_rms_norm_i16(s_ch_gcc[1], frames)),
                           fmaxf(calc_rms_norm_i16(s_ch_gcc[2], frames),
                                 calc_rms_norm_i16(s_ch_gcc[3], frames)));
        float doa_energy = fmaxf(brms, transient_rms * 0.50f);
        bool source_active = update_source_active(brms, transient_rms,
                                                  srp_valid, doa_result.confidence);
        /* 持续声源 (音乐/说话 ~1.5s 以上): 拉长 SRP 谱积分抗反射游走;
         * 瞬态或新出现的声源保持默认快速积分 */
        if (source_active && transient_rms < NF_SNAP_TRANSIENT_MIN) {
            if (s_sustained_frames < 1000U) {
                s_sustained_frames++;
            }
        } else {
            s_sustained_frames = 0;
        }
        gcc_phat_set_score_ema_alpha(
            s_sustained_frames > 70U ? 0.15f : 0.33f);
        float transient_peak_ratio = rms > 0.0f ? transient_rms / rms : 0.0f;
        bool strong_snap = doa_event_is_strong_snap(
            rms, transient_rms, transient_peak_ratio,
            doa_result.score);
        bool mirror_ambiguous = doa_result.valid &&
                                doa_result.mirror_ratio >= TRANSIENT_MIRROR_RATIO_MAX;
        /* 近场削顶通道: 强瞬态 + ADC 削顶 + SRP 不可信时, 用 onset 球面波拟合
         * 给方向 (远场 SRP 与互相关在削顶+近场下全部失效) */
        int clip_count = count_clipped(s_ch[0], frames) +
                         count_clipped(s_ch[1], frames) +
                         count_clipped(s_ch[2], frames) +
                         count_clipped(s_ch[3], frames);
        bool chain_clipped = clip_count > frames / NF_CLIP_SAMPLES;
        bool srp_usable = srp_valid && !chain_clipped &&
                          doa_result.score >= DOA_TRACK_MIN_SCORE;
        bool nf_transient = false;
        if (transient_rms >= NF_SNAP_TRANSIENT_MIN && chain_clipped && !srp_usable) {
            const int16_t *nf_ch[CHAN_NUM] = {
                s_ch[0], s_ch[1], s_ch[2], s_ch[3]
            };
            float od_angle = 0.0f, od_residual = 0.0f;
            if (onset_doa_estimate(nf_ch, frames, &od_angle, &od_residual)) {
                angle_raw = direction_calibrate_doa(od_angle);
                nf_transient = true;
            }
        }
        bool transient_doa = !mirror_ambiguous &&
                             (doa_event_use_fast_path(
                                  rms, transient_rms, transient_peak_ratio,
                                  doa_result.score, doa_result.confidence) ||
                              nf_transient);
        const char *event_tag = strong_snap ? " snap"
            : (nf_transient ? " nfsnap" : (transient_doa ? " tap" : ""));
        float tracker_confidence = transient_doa &&
                                   doa_result.confidence < TRANSIENT_DOA_CONF_BOOST
            ? TRANSIENT_DOA_CONF_BOOST
            : doa_result.confidence;
        if (nf_transient) {
            /* onset 拟合质量已由残差门限把关, 置信高于对侧镜像门限,
             * 否则 20cm 响指换新方向会被对侧保护拦成"不亮灯" */
            tracker_confidence = NF_SNAP_CONFIDENCE;
        }
        float track_min_energy = fmaxf(DOA_TRACK_MIN_ENERGY_ABS,
                                       s_rms_floor * DOA_TRACK_MIN_ENERGY_RATIO);
        bool srp_quality_ok = srp_valid &&
                              doa_result.score >= DOA_TRACK_MIN_SCORE;
        bool localization_valid = source_active &&
                                  ((srp_quality_ok && doa_energy >= track_min_energy &&
                                    doa_result.confidence >= DOA_TRACK_MIN_CONF) ||
                                   /* 近场削顶 onset 通道: 无 SRP 质量也认可 */
                                   transient_doa);
        float angle = s_doa_angle;
        bool tracker_updated;
        if (transient_doa && localization_valid) {
            tracker_updated = doa_tracker_update_transient(
                &s_doa_tracker, angle_raw, tracker_confidence,
                true, &angle);
        } else {
            tracker_updated = doa_tracker_update(
                &s_doa_tracker, angle_raw, tracker_confidence,
                localization_valid, &angle);
        }

#elif APP_MODE == APP_MODE_SOUND_CLASSIFY

        /* ---- 音色识别: 填充 EI 推理窗口 (不阻塞) ---- */
        if (s_ei_fill_index < 0) {
            uint8_t free_index = 0;
            if (xQueueReceive(s_ei_free_queue, &free_index, 0) == pdTRUE) {
                s_ei_fill_index = free_index;
                s_ei_pos = 0;
                s_ei_energy_sum = 0.0;
                s_ei_energy_count = 0;
                ei_decim_reset();
                s_ei_src_ch = choose_ei_channel(rms_ch, CHAN_NUM, s_ei_src_ch);
            }
        }

        if (s_ei_fill_index >= 0) {
            float src_rms = rms_ch[s_ei_src_ch];
            float target_gain = src_rms >= EI_RMS_MIN_FOR_AGC
                ? EI_TARGET_RMS / src_rms : 1.0f;
            if (target_gain < EI_GAIN_MIN) target_gain = EI_GAIN_MIN;
            if (target_gain > EI_GAIN_MAX) target_gain = EI_GAIN_MAX;
            if (!s_agc_valid) {
                s_ei_gain = target_gain;
                s_agc_valid = true;
            } else {
                s_ei_gain = EI_GAIN_SMOOTH_ALPHA * target_gain +
                    (1.0f - EI_GAIN_SMOOTH_ALPHA) * s_ei_gain;
            }

            /* 48kHz 原始数据经抗混叠抽取 + 增益后写入 EI 窗口.
             * FIR 归一化, 抽取前后 RMS 不变, 能量统计按 16k 样本数加权. */
            int n_copy = ei_decim_push(&s_ei_buf[s_ei_fill_index][s_ei_pos],
                                       EI_SAMPLES - s_ei_pos,
                                       s_ch[s_ei_src_ch], frames, s_ei_gain);
            s_ei_energy_sum += (double)src_rms * (double)src_rms * (double)n_copy;
            s_ei_energy_count += (uint32_t)n_copy;
            s_ei_pos += n_copy;

            if (s_ei_pos >= EI_SAMPLES) {
                inference_job_t job = {
                    .buffer_index = (uint8_t)s_ei_fill_index,
                    .source_channel = (uint8_t)s_ei_src_ch,
                    .window_rms = s_ei_energy_count > 0
                        ? (float)sqrt(s_ei_energy_sum / (double)s_ei_energy_count)
                        : 0.0f,
                    .gain = s_ei_gain,
                };
                if (xQueueSend(s_ei_ready_queue, &job, 0) != pdTRUE) {
                    uint8_t free_index = job.buffer_index;
                    xQueueSend(s_ei_free_queue, &free_index, 0);
                    ESP_LOGW(TAG, "Inference queue full; classification window dropped");
                }
                s_ei_fill_index = -1;
            }
        }

#endif /* APP_MODE == APP_MODE_SOUND_CLASSIFY */

#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
        /* ---- 更新共享结果 (互斥) ---- */
        if (xSemaphoreTake(s_result_mtx, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_doa_angle = angle;
            xSemaphoreGive(s_result_mtx);
        }

        float display_angle = angle;
        bool display_active = doa_display_gate_update(
            &s_doa_display_gate, tracker_updated, source_active,
            localization_valid, doa_energy, angle, &display_angle);

        /* ---- LED 方向指示 (带熄灭保持) ----
         * LED 仅与声源有效性相关, 不受鸟叫识别门控影响.
         * 熄灭需失活持续 LED_OFF_HOLD_FRAMES, 防门限边缘快速亮灭闪烁 */
        bool led_active = display_active && s_doa_tracker.initialized;
        if (led_active) {
            s_led_hold_frames = 0;
            s_led_lit = true;
        } else if (s_led_hold_frames < LED_OFF_HOLD_FRAMES) {
            s_led_hold_frames++;
        } else {
            s_led_lit = false;
        }
        led_update_direction(display_angle, s_led_lit);
#endif

        frame_idx++;
#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
        if ((frame_idx % 8U) == 0U) {
            ESP_LOGI(TAG,
                     "DOA=%.1f° raw=%.1f° rms=[%.4f %.4f %.4f %.4f] xc=[%.2f %.2f %.2f] "
                     "tr=%.4f pr=%.2f srp=%.3f second=%.3f conf=%.3f mir=%.2f br=%.4f fl=%.4f "
                     "act=%d dsp=%d oc=%d%s mics=0x%02x pairs=%u status=%s",
                     angle, angle_raw, rms0, rms1, rms2, rms3,
                     xc01, xc02, xc03,
                     transient_rms, transient_peak_ratio,
                     doa_result.score, doa_result.second_score,
                     doa_result.confidence, doa_result.mirror_ratio,
                     brms, s_rms_floor, (int)source_active, (int)display_active, clip_count,
                     event_tag,
                     doa_result.active_mic_mask,
                     doa_result.active_pair_count,
                     tracker_updated ? "valid" : "rejected");
        }
#endif /* APP_MODE == APP_MODE_SOUND_LOCALIZATION */

        /* ---- 推送数据到 Web 客户端 ---- */
        TickType_t now = xTaskGetTickCount();
        if ((now - last_ui_push_tick) >= pdMS_TO_TICKS(UI_PUSH_PERIOD_MS)) {
            last_ui_push_tick = now;
#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
            float bird_ui = 0.0f;
            float noise_ui = 1.0f;
            bool detected_ui = false;
            if (xSemaphoreTake(s_result_mtx, pdMS_TO_TICKS(10)) == pdTRUE) {
                bird_ui = s_bird_conf;
                noise_ui = s_noise_conf;
                detected_ui = s_bird_detected;
                xSemaphoreGive(s_result_mtx);
            }
            bool bird_gate_ui = doa_gate_is_open(detected_ui, bird_ui);
            /* 推送门控后的显示角度 (与 LED 一致, 带死区), 原始跟踪角抖动大,
             * 直接推送会让网页罗盘数字/指针跟着估计噪声逐帧晃动 */
            wifi_server_push_data(display_angle,
                                  bird_ui,
                                  noise_ui,
                                  detected_ui && bird_gate_ui);
#else /* 音色识别模式: 无罗盘, 推送识别结果 */
            float bird_ui = 0.0f;
            float noise_ui = 1.0f;
            bool detected_ui = false;
            if (xSemaphoreTake(s_result_mtx, pdMS_TO_TICKS(10)) == pdTRUE) {
                bird_ui = s_bird_conf;
                noise_ui = s_noise_conf;
                detected_ui = s_bird_detected;
                xSemaphoreGive(s_result_mtx);
            }
            wifi_server_push_data(0.0f, bird_ui, noise_ui, detected_ui);
            led_update_detection(detected_ui);
#endif
        }

        /* WDT 安全网: 仅当本帧纯处理时间超过实时预算 (约 21.3ms) 时才让出
         * 1 tick, 给 IDLE1 喂任务看门狗的机会; 处理超前时下一帧的
         * i2s_channel_read 会自然阻塞让出 CPU, 不影响吞吐与精度. */
        if ((esp_timer_get_time() - frame_start_us) >= 21000LL) {
            vTaskDelay(1);
        }
    }
}

/* ================================================================ */
/*  app_main                                                         */
/* ================================================================ */
void app_main(void)
{
#ifdef RUN_EXAMPLE_I2C_SCAN
    example_i2c_scan_start();   /* 不返回 */
#elif defined(RUN_EXAMPLE_ADC)
    example_adc_start();        /* 不返回 */
#elif defined(RUN_EXAMPLE_UART)
    example_uart_start();       /* 不返回 */
#elif defined(RUN_EXAMPLE_RGB)
    example_rgb_start();        /* 不返回 */
#elif defined(RUN_EXAMPLE_ES7210)
    example_es7210_start();     /* 不返回 */
#elif defined(RUN_EXAMPLE_USB)
    example_usb_start();        /* 不返回 */
#elif defined(RUN_EXAMPLE_BLE)
    example_ble_start();        /* 不返回 */
#elif defined(RUN_EXAMPLE_BOARD_TEST)
    ESP_LOGI(TAG, ">>> 板级自检模式 (RUN_EXAMPLE_BOARD_TEST)");
    board_test_start();
    vTaskDelete(NULL);
#else
    run_bird_doa_demo();
#endif
}

static void run_bird_doa_demo(void)
{
    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, " Bird Sound Detection System");
    ESP_LOGI(TAG, " ESP32-S3 + ES7210 + 4-Mic");
    ESP_LOGI(TAG, " Array r=40mm, Fs=%d Hz", SAMPLE_RATE);
    ESP_LOGI(TAG, "==============================");
#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
    ESP_LOGI(TAG, " 模式: 声源定位 (DOA)");
#elif APP_MODE == APP_MODE_SOUND_CLASSIFY
    ESP_LOGI(TAG, " 模式: 音色识别 (Classify)");
#endif

#ifdef ENABLE_BOARD_TEST
    /* 兼容旧开关: 板级自检模式 */
    ESP_LOGI(TAG, ">>> 板级自检模式 (ENABLE_BOARD_TEST)");
    board_test_start();
    ESP_LOGI(TAG, ">>> 自检已在独立任务启动, 如需恢复正常检测请注释 board_test.h 中的 ENABLE_BOARD_TEST");
    vTaskDelete(NULL);
#endif

    s_result_mtx = xSemaphoreCreateMutex();
    configASSERT(s_result_mtx != NULL);
#if APP_MODE == APP_MODE_SOUND_CLASSIFY
    s_ei_ready_queue = xQueueCreate(EI_BUFFER_COUNT, sizeof(inference_job_t));
    s_ei_free_queue = xQueueCreate(EI_BUFFER_COUNT, sizeof(uint8_t));
    configASSERT(s_ei_ready_queue != NULL);
    configASSERT(s_ei_free_queue != NULL);
    for (uint8_t i = 0; i < EI_BUFFER_COUNT; ++i) {
        configASSERT(xQueueSend(s_ei_free_queue, &i, 0) == pdTRUE);
    }
#endif

    /* 硬件初始化 */
    init_i2s();
    esp_err_t es7210_ret = init_es7210();
    if (es7210_ret != ESP_OK) {
        ESP_LOGE(TAG, "ES7210 初始化失败: %s", esp_err_to_name(es7210_ret));
        ESP_LOGE(TAG, "系统将保持在线但不启动音频任务, 请先修复 I2C 连线/地址");
    }
    led_init();

    /* 算法与模型初始化 (按模式) */
#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
    gcc_phat_init(FRAME_LEN);
    doa_tracker_init(&s_doa_tracker, 0.025f);
    doa_display_gate_init(&s_doa_display_gate);
    wifi_server_set_mode("doa");
#elif APP_MODE == APP_MODE_SOUND_CLASSIFY
    bird_decision_init(&s_bird_decision);
    ei_inference_init();
    wifi_server_set_mode("cls");
#endif

    /* 网络初始化 (阻塞直到 Wi-Fi 连接成功) */
    wifi_server_init();

    /* 启动音频处理任务, 固定在 Core 1 运行
     * Core 0 留给 Wi-Fi 协议栈 */
    if (es7210_ret == ESP_OK) {
#if APP_MODE == APP_MODE_SOUND_CLASSIFY
        xTaskCreatePinnedToCore(
            inference_task, "inference_task",
            INFERENCE_TASK_STACK, NULL,
            INFERENCE_TASK_PRIO, NULL, 0);
#endif
        xTaskCreatePinnedToCore(
            audio_task, "audio_task",
            AUDIO_TASK_STACK, NULL,
            AUDIO_TASK_PRIO, NULL, 1);
    }

    ESP_LOGI(TAG, "系统已启动, 打开 http://<device_ip> 查看面板");
}