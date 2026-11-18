/*
 * 总合演示: 鸟声检测 + 声源定位
 *
 * 硬件: ESP32-S3 + ES7210 + 4x SPH1642HT5H-1，半径40mm/对径80mm十字阵列。
 * 功能:
 *   1. 4通道 I2S TDM 音频同步采集 (48kHz)
 *   2. GCC-PHAT + SRP 声源定位 (方位角估计, 48kHz 全速率处理)
 *   3. Edge Impulse 鸟声/噪声分类 (TFLite Micro, INT8, 16kHz 输入经 3:1 抗混叠抽取)
 *   4. 16 颗 WS2812：单方向呼吸/移动跟随/可信双方向同时显示；网页主方向。
 *
 * 数据流：I2S同步采集 → 分通道滤波 → SRP空间图 → 主方向跟踪/显示回差
 *         → doa_led_scene（双峰确认）→ 最新场景邮箱 → Core0独立50Hz灯效。
 * 单源灵敏度沿用已验收值；只想改外观请编辑 led/led_effects_config.h。
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
#include "doa/doa_led_scene.h"
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
#ifdef DOA_PCM_STREAM
#include "doa/doa_pcm_stream.h"
#endif

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

/* 半径40mm，对径80mm；最大时差80mm/343m/s≈233µs，48kHz下约11.2样本。
 * 四通道必须同频、同步，不能各自异步采样。当前定位频带300Hz~6kHz。
 * 分类模型仍用16kHz，通过3:1抗混叠抽取得到；改采样率须重算滤波系数。 */
#define SAMPLE_RATE                     (48000)                            /* 采集Hz；定位时差及滤波均依赖此值 */
#define EI_SAMPLE_RATE                  (16000)                            /* 分类输入Hz，仅分类模式使用 */
#define SAMPLE_BITS                     (I2S_DATA_BIT_WIDTH_16BIT)         /* 每通道16bit有符号PCM */
#define CHAN_NUM                        (4)                                /* 同时采样的通道数，须与阵列坐标/插槽数一致 */
#define MCLK_MULTIPLE                   (I2S_MCLK_MULTIPLE_256)            /* 主时钟/采样率比，48kHz×256=12.288MHz */
#define TDM_SLOT_MASK                   (I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3) /* TDM四个插槽，顺序就是s_ch[0..3] */
#define I2S_FMT                         (ES7210_I2S_FMT_I2S)               /* ES7210与ESP32两端均为Philips I2S格式 */

/* ES7210 编解码器设置 */
#define ES7210_ADDR                     (0x40)                             /* 首选7位I2C地址，不是左移后的写地址 */
#define ES7210_ADDR_MIN                 (0x40)                             /* 探测起点，7位地址 */
#define ES7210_ADDR_MAX                 (0x43)                             /* 探测终点，包含本地址 */
#define ES7210_I2C_CLK_HZ               (100000)                           /* I2C时钟Hz；100kHz提高布线容错 */
#define ES7210_GAIN                     (ES7210_MIC_GAIN_30DB)             /* 模拟麦增益30dB；增大同时增加削顶风险 */
#define ES7210_BIAS                     (ES7210_MIC_BIAS_2V87)             /* 麦克风偏置电压2.87V */
/* 数字音量+12dB，保留当前实机值。提高会同时放大声音和噪声，不能凭空
 * 改善模拟信噪比，且强瞬态更容易削顶；不要用数字音量代替声学定位校准。 */
#define ES7210_VOL                      (12)                               /* 数字音量dB，保持已验收值+12dB */

/* ================================================================ */
/*  处理参数                                                         */
/* ================================================================ */

/* GCC-PHAT 分析帧长 (对应 GCC_FFT_SIZE): 1024 点 @48kHz = 21.3ms.
 * 帧率 46.9fps, 是原 16kHz 配置的 3 倍, 下列按帧更新的平滑系数均已换算. */
#define FRAME_LEN                       (1024)                             /* 每通道点数；1024/48000=21.33ms */

/* EI 模型单次推理所需采样数 */
#define EI_SAMPLES                      (EI_CLASSIFIER_RAW_SAMPLE_COUNT)   /* 模型所需16kHz采样数，由模型元数据决定 */

/* I2S 每次读取的 int16 样本数 = FRAME_LEN * 4ch */
#define I2S_BUF_SAMPLES                 (FRAME_LEN * CHAN_NUM)             /* 交织int16数量=1024点×4通道 */

/* 分类稳定化参数: 抑制噪音误判为鸟叫 */
#define CLS_SMOOTH_ALPHA                (0.55f)                            /* 分类置信EMA系数，越大越跟随本次推理 */
#define EI_WINDOW_MIN_RMS               (0.0015f)                          /* 低于此归一化RMS视作静音，不跑模型 */

/* EI 输入增益控制: 提升弱鸟叫召回, 并限制过放大 */
#define EI_TARGET_RMS                   (0.045f)                           /* 分类AGC期望RMS，不影响定位与LED声强 */
#define EI_RMS_MIN_FOR_AGC              (0.0012f)                          /* 过弱背景不做大幅AGC放大 */
#define EI_GAIN_MIN                     (0.70f)                            /* 分类线性增益下限，不是dB */
#define EI_GAIN_MAX                     (6.00f)                            /* 分类线性增益上限，防止放大噪声 */
/* 0.09 @21.3ms 帧 ≈ 原 0.25 @64ms 帧的时间常数 (~200ms) */
#define EI_GAIN_SMOOTH_ALPHA            (0.09f)                            /* 分类输入增益EMA，约226ms时间常数 */

/* 声源门控：RMS均以int16满量程归一化，0.001≈-60dBFS。
 * 以下“倍率”为RMS幅度比，不是功率比，dB值需20*log10(倍率)。
 * 开启门槛=max(绝对值,噪声底*倍率)，关闭门槛更低，避免门槛边缘亮灭。 */
#define RMS_FLOOR_MIN                   (0.00015f)                         /* 归一化噪声底下限，避免除零或过度灵敏 */
/* EMA系数0~1，越大变化越快；当前帧21.33ms，下降系数.06约345ms时间常数。
 * 已激活声音不再上调噪声底，防止持续音乐被当成背景学掉。 */
#define RMS_FLOOR_DECAY_ALPHA           (0.06f)                            /* 噪声底下降EMA，环境变安静时快速恢复 */
#define LED_SNR_STRONG_RATIO            (2.4f)                             /* 强声开启的带内RMS/背景RMS比 */
#define LED_SNR_STRONG_ABS_MIN          (0.0006f)                          /* 强声开启绝对RMS下限 */
#define LED_SNR_WEAK_RATIO              (1.8f)                             /* 弱声开启RMS比，另需空间置信证据 */
#define LED_SNR_WEAK_ABS_MIN            (0.0003f)                          /* 弱声开启绝对RMS下限 */
#define LED_SNR_OFF_RATIO               (1.35f)                            /* 关闭RMS比，低于开启值形成回差 */
#define LED_SNR_OFF_ABS_MIN             (0.00025f)                         /* 关闭绝对RMS门槛 */
#define LED_WEAK_SRP_CONF_THRESHOLD     (0.05f)                            /* 普通弱声开启最低空间置信度 */
/* 噪声底上升极慢 (~27s @46.9fps): 适应环境变吵; 下降快 (见更新逻辑).
 * 组合即"最小统计量"噪声估计: 任何声音的间隙都会暴露真实底噪,
 * floor 自动落到最低处; 瞬态尖峰来不及抬升 floor. */
#define RMS_FLOOR_RISE_ALPHA            (0.0008f)                          /* 未激活时噪声底上升EMA，约26.7s时间常数 */

/* 跟踪有效性 (localization_valid) 的能量门限: 同样改为带内 SNR 自适应 */
#define DOA_TRACK_MIN_ENERGY_ABS        (0.0004f)                          /* 普通跟踪最低带内RMS，弱空间证据可走补充路径 */
#define DOA_TRACK_MIN_ENERGY_RATIO      (1.6f)                             /* 普通跟踪RMS/背景比 */

/* Foreground-weighted SRP score floor, combined with adaptive energy,
 * peak competition and opposite-direction checks. */
#define DOA_TRACK_MIN_CONF              (0.025f)                           /* 主方向最低置信；沿用已调好的值 */
#define DOA_TRACK_MIN_SCORE             (0.006f)                           /* 主方向相关分数下限，非声压值 */

/* 小信号时四路 RMS 很接近, 频繁切 EI 输入通道会让模型输入音色抖动 */
#define EI_CHANNEL_SWITCH_RATIO         (1.35f)                            /* 最强通道超过当前通道35%才切分类输入 */

/* Short transients are diluted by the 1024-sample full-frame RMS. */
#define TRANSIENT_BLOCK_SAMPLES         (96)                               /* 短块点数96=2ms，用于检测短促声峰值 */
/* Strong energy-only activation; weaker events need spatial evidence. */
#define TRANSIENT_RMS_ON_THRESHOLD      (0.0080f)                          /* 强瞬态直接激活最低归一化RMS */

/* 镜像含糊门限: SRP 对侧 (±180°) 分数达到主峰该比例时, 本帧方向视为含糊,
 * 瞬态 (响指) 不走 1~2 帧快速通道, 退回连续帧确认, 避免响指亮到对面.
 * 十字阵列中心对称, 对径麦 80mm 在 >2.14kHz 空间混叠, 响指/拍手能量
 * 恰好集中在该混叠频段, 是对侧误亮的主要来源. */
#define TRANSIENT_MIRROR_RATIO_MAX      (0.60f)                            /* 瞬态对侧分数/主分数上限，越小越保守 */

/* RMS 平滑系数: 0.11 @21.3ms 帧 ≈ 原 0.30 @64ms 帧的时间常数 */
#define RMS_SMOOTH_ALPHA                (0.11f)                            /* 源音量EMA，约183ms时间常数 */

/* Matched per-channel 300 Hz high-pass / 6 kHz low-pass at 48 kHz. */
#define GCC_HP_ALPHA                    (0.962f)                           /* 一阶高通系数，48kHz下约300Hz */
#define GCC_LP_ALPHA                    (0.440f)                           /* 一阶低通系数，48kHz下约6kHz */

/* 仅约束网页的“鸟叫检测”标志；声源定位/灯环并不受鸟叫分类结果限制。 */
#define DOA_BIRD_CONF_GATE              (0.40f)                            /* 网页鸟叫标志最低分类置信，不门控方向灯 */

/* 网页角度推送节奏 (ms) */
#define UI_PUSH_PERIOD_MS               (100)                              /* 网页SSE推送间隔毫秒，灯效有独立20ms周期 */

/* 任务参数 */
#define AUDIO_TASK_STACK                (8192)                             /* 音频任务栈字节数，ESP-IDF的任务栈参数单位为字节 */
#define AUDIO_TASK_PRIO                 (5)                                /* 音频优先级5，高于动画与日志 */
#define INFERENCE_TASK_STACK            (8192)                             /* 分类任务栈字节数 */
#define INFERENCE_TASK_PRIO             (3)                                /* 分类任务优先级3 */
#define EI_BUFFER_COUNT                 (1)                                /* 分类音频缓冲个数=1，推理期间不覆写 */

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
static doa_led_scene_t s_doa_led_scene;
static float s_effect_direction; /* 只供灯效的连续主跟踪角；不改对外DOA/网页角度 */

/* 音频缓冲区 */
static int16_t s_i2s_buf[I2S_BUF_SAMPLES];           /* I2S TDM 交织原始数据 */
static int16_t s_ch[CHAN_NUM][FRAME_LEN];             /* 解交织后各通道数据 */
#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
static int16_t s_ch_gcc[CHAN_NUM][FRAME_LEN];         /* GCC 预滤波输入 */
#endif
#if APP_MODE == APP_MODE_SOUND_CLASSIFY
static int16_t s_ei_buf[EI_BUFFER_COUNT][EI_SAMPLES]; /* 当前单缓冲，由空闲/就绪队列移交所有权 */
#endif
static int     s_ei_pos = 0;                           /* 当前写入位置 */
static int     s_ei_fill_index = -1;
static float   s_rms_smooth = 0.0f;
static bool    s_rms_valid = false;
static float   s_rms_floor = 0.0010f;   /* 环境噪声底跟踪 (自适应门限基准) */
static bool    s_source_active = false;
static unsigned s_sustained_frames = 0; /* 连续有声帧数；普通音乐节拍不清零 */
/* LED 熄灭保持: 显示失活后须持续这么久才真正熄灯, 防止门限边缘
 * 1 秒内亮灭数次 (抽搐式闪烁). 亮起无延迟. */
#define LED_OFF_HOLD_FRAMES             45U                                /* 保留原熄灭保持45帧≈960ms，之后灯效再淡出 */
static unsigned s_led_hold_frames = 0;
static bool     s_led_lit = false;
static unsigned s_source_idle_frames = 0;
static unsigned s_source_hold_frames = 0;
static float s_previous_frame_rms = 0.0f;
static float s_previous_transient_rms = 0.0f;
static volatile uint32_t s_rx_overflows = 0;
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

#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
typedef struct {
    float angle, raw, rms[4], transient, peak_ratio;
    float score, second, confidence, mirror, band_rms, floor;
    float instant_angle, instant_score, instant_confidence, instant_mirror;
    bool active, display, updated;
    int clips, pixel;
    uint8_t mic_mask, pairs;
    long processing_us;
    uint32_t overflows;
    uint32_t frame_ms;
    unsigned sustained_frames;
    bool instant_analysis;
    bool led_lit;
    unsigned spatial_peaks, visible_sources;
    const char *event;
} doa_diagnostic_t;
static QueueHandle_t s_doa_log_queue;

/* UART/USB writes and float formatting must never block the I2S consumer. */
static void doa_log_task(void *arg)
{
    (void)arg;
    doa_diagnostic_t d;
    while (xQueueReceive(s_doa_log_queue, &d, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(TAG,
                 "DOA=%.1f° raw=%.1f° rms=[%.4f %.4f %.4f %.4f] "
                 "tr=%.4f pr=%.2f srp=%.3f second=%.3f conf=%.3f mir=%.2f br=%.4f fl=%.4f "
                 "inst=[%.1f %.3f %.3f %.2f] act=%d dsp=%d oc=%d%s "
                 "mics=0x%02x pairs=%u led=%d us=%ld ovf=%lu frame_ms=%lu fast=%d steady=%u lit=%d peaks=%u sources=%u status=%s",
                 d.angle, d.raw, d.rms[0], d.rms[1], d.rms[2], d.rms[3],
                 d.transient, d.peak_ratio, d.score, d.second, d.confidence, d.mirror,
                 d.band_rms, d.floor, d.instant_angle, d.instant_score,
                 d.instant_confidence, d.instant_mirror,
                 (int)d.active, (int)d.display, d.clips, d.event,
                 d.mic_mask, d.pairs, d.pixel, d.processing_us,
                 (unsigned long)d.overflows, (unsigned long)d.frame_ms,
                 (int)d.instant_analysis, d.sustained_frames, (int)d.led_lit,
                 d.spatial_peaks, d.visible_sources,
                 d.updated ? "valid" : "rejected");
    }
}
#endif

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

static bool on_rx_overflow(i2s_chan_handle_t channel, i2s_event_data_t *event, void *ctx)
{
    (void)channel;
    (void)event;
    (void)ctx;
    s_rx_overflows++;
    return false;
}

/* ================================================================ */
/*  I2S TDM 初始化                                                   */
/* ================================================================ */
static void init_i2s(void)
{
    ESP_LOGI(TAG, "I2S TDM 初始化 (%d Hz, %d ch)", SAMPLE_RATE, CHAN_NUM);

    i2s_chan_config_t rx_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    /* 96 ms of DMA headroom for occasional USB/Wi-Fi scheduling stalls. */
    rx_cfg.dma_desc_num = 12;
    rx_cfg.dma_frame_num = 384;
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
    i2s_event_callbacks_t callbacks = {.on_recv_q_ovf = on_rx_overflow};
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(s_i2s_rx, &callbacks, NULL));
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

/* Diagnostic zero-lag correlation. Low/negative values also occur normally
 * for spatially separated microphones; they do not prove reversed polarity. */
static float zero_lag_corr(const int16_t *a, const int16_t *b, int n)
{
    if (a == NULL || b == NULL || n <= 0) {
        return 0.0f;
    }
    /* ESP32-S3 has float hardware but emulates double in software. */
    float sa = 0.0f, sb = 0.0f, sab = 0.0f;
    for (int i = 0; i < n; ++i) {
        float x = (float)a[i];
        float y = (float)b[i];
        sa += x * x;
        sb += y * y;
        sab += x * y;
    }
    if (sa <= 0.0 || sb <= 0.0) {
        return 0.0f;
    }
    return sab / sqrtf(sa * sb);
}

/* ---- 近场瞬态 onset 定位 (抗 ADC 削顶) ----
 * 满量程仅对应 ~64dB SPL, 20cm 级响指/拍手在麦克风处可达 95dB+, 必然硬削顶;
 * 削顶波形做互相关/SRP 全是垃圾 (表现为"一个灯都不亮"或角度乱跳), 且
 * 远场平面波假设在 <0.5m 时 TDOA 误差可达 1~2 采样 (~30°).
 * 改用各通道能量累积起点 (onset) 估计 TDOA: 到达先后不受削顶影响;
 * 再用球面波模型 (近场) 在 方位×距离 小网格上拟合方向. */
#define ONSET_PEAK_FRAC                 (0.30f)                            /* 削顶近场到达检测比例0.30，越低越易受先行噪声影响 */
#define ONSET_MAX_RESIDUAL              (2.5f)                             /* 近场拟合残差上限，单位采样点 */
#define NF_CLIP_SAMPLES                 (2)                                /* 削顶样本阈值分母：四路总削顶数>帧长/2 */
#define NF_SNAP_TRANSIENT_MIN           (0.0100f)                          /* 近场削顶通道最低瞬态RMS */
#define NF_SNAP_CONFIDENCE              (0.65f)                            /* 近场拟合通过后使用的置信值，并非概率 */
static const float s_nf_mic_pos_m[CHAN_NUM][2] = {
    {0.0f, -MIC_ARRAY_RADIUS_MM / 1000.0f}, {-MIC_ARRAY_RADIUS_MM / 1000.0f, 0.0f},
    {MIC_ARRAY_RADIUS_MM / 1000.0f, 0.0f}, {0.0f, MIC_ARRAY_RADIUS_MM / 1000.0f},
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
    int thresh = (int)((float)peak * ONSET_PEAK_FRAC);
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
#define EI_DECIM_FACTOR                 (SAMPLE_RATE / EI_SAMPLE_RATE)     /* 48kHz→16kHz整数抽取比=3 */
#define EI_DECIM_TAPS                   (12)                               /* 分类抗混叠FIR抽头数12，非定位滤波器 */
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
    } else if (!s_source_active) {
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
    coherent_weak_source |= brms_raw >= weak_gate && srp_valid &&
                            srp_confidence >= 0.16f;

    if (strong_source || coherent_weak_source) {
        s_source_active = true;
        s_source_hold_frames = 6U;
    } else if (s_source_hold_frames > 0U) {
        s_source_hold_frames--;
    } else if (s_rms_smooth <= off_gate) {
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
    TickType_t last_forced_yield = 0;
    bool last_source_active = false;
    unsigned onset_log_frames = 0;
    uint32_t last_rx_overflows = 0;

    while (1) {
        size_t bytes_read = 0;

        /* 读取一帧 4 通道 TDM 数据 */
        esp_err_t ret = i2s_channel_read(
            s_i2s_rx, s_i2s_buf, sizeof(s_i2s_buf),
            &bytes_read, pdMS_TO_TICKS(500));
        if (ret != ESP_OK || bytes_read == 0) {
            continue;
        }
#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
        if (s_rx_overflows != last_rx_overflows) {
            /* A DMA queue drop can splice non-adjacent sample blocks into
             * this frame. Never use it to change the indicated direction. */
            last_rx_overflows = s_rx_overflows;
            gcc_phat_reset_history();
            memset(s_gcc_filter, 0, sizeof(s_gcc_filter));
            continue;
        }
#endif

        /* 测量点: 在阻塞读之后, 只统计纯处理时间, 排除读数据阻塞 */
        int64_t frame_start_us = esp_timer_get_time();

        int frames = bytes_read / (CHAN_NUM * sizeof(int16_t));
        deinterleave(s_i2s_buf, frames);

#ifdef DOA_PCM_DUMP
        /* Explicit diagnostic build only: dump one simultaneous 4ch frame.
         * Normal localization is paused because UART hex output is slow. */
        if (++frame_idx >= 24U) {
            ESP_LOGI("PCM", "BEGIN frames=%d fs=%d", frames, SAMPLE_RATE);
            ESP_LOG_BUFFER_HEX_LEVEL("PCM", s_i2s_buf, bytes_read, ESP_LOG_INFO);
            ESP_LOGI("PCM", "END");
            frame_idx = 0;
        }
        continue;
#endif

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
        float transient0 = calc_peak_block_rms_norm_i16(s_ch[0], frames, TRANSIENT_BLOCK_SAMPLES);
        float transient1 = calc_peak_block_rms_norm_i16(s_ch[1], frames, TRANSIENT_BLOCK_SAMPLES);
        float transient2 = calc_peak_block_rms_norm_i16(s_ch[2], frames, TRANSIENT_BLOCK_SAMPLES);
        float transient3 = calc_peak_block_rms_norm_i16(s_ch[3], frames, TRANSIENT_BLOCK_SAMPLES);
        float transient_rms = fmaxf(fmaxf(transient0, transient1), fmaxf(transient2, transient3));
        float transient_peak_ratio = rms > 0.0f ? transient_rms / rms : 0.0f;
        gcc_phat_result_t doa_result = {0};
        bool analyze_instant = doa_event_needs_instant_analysis(
            s_source_active, rms, s_previous_frame_rms, transient_rms,
            s_previous_transient_rms, s_rms_floor);
        gcc_phat_set_instant_enabled(analyze_instant);
        s_previous_frame_rms = rms;
        s_previous_transient_rms = transient_rms;
        gcc_phat_set_noise_learning(!s_source_active);
        /* Clear the old source once after silence. Repeating this every idle
         * frame would prevent weak coherent observations from integrating. */
        if (s_source_idle_frames == 6U) gcc_phat_reset_history();
        bool srp_valid = gcc_phat_estimate_4ch(
            doa_mics, frames, SAMPLE_RATE, &doa_result);
        float angle_raw = direction_calibrate_doa(doa_result.angle_deg);
        /* Filtered RMS for adaptive source activation and localization gating. */
        float brms = fmaxf(fmaxf(calc_rms_norm_i16(s_ch_gcc[0], frames),
                                 calc_rms_norm_i16(s_ch_gcc[1], frames)),
                           fmaxf(calc_rms_norm_i16(s_ch_gcc[2], frames),
                                 calc_rms_norm_i16(s_ch_gcc[3], frames)));
        float doa_energy = fmaxf(brms, transient_rms * 0.50f);
        bool weak_evidence = doa_event_has_weak_spatial_evidence(
            srp_valid, brms, s_rms_floor, doa_result.score,
            doa_result.zero_delay_score, doa_result.confidence, doa_result.mirror_ratio);
        bool source_active = update_source_active(brms, transient_rms,
                                                  srp_valid, doa_result.confidence);
        if (weak_evidence) {
            s_source_active = source_active = true;
            s_source_hold_frames = 6U;
        }
        if (source_active) {
            s_source_idle_frames = 0;
        } else if (s_source_idle_frames < 15U) {
            if (++s_source_idle_frames == 15U) {
                /* A new source after silence may legitimately be opposite. */
                doa_tracker_init(&s_doa_tracker, DOA_TRACK_MIN_CONF);
                s_doa_tracker.acquisition_frames = 5U;
            }
        }
        /* 连续有效70帧≈1.49s后用较慢空间积分；明确能量突升时临时加快。
         * 这里只选EMA系数，不增加固定等待时间，不抹掉原始采集帧。 */
        if (source_active) {
            if (s_sustained_frames < 1000U) {
                s_sustained_frames++;
            }
        } else {
            s_sustained_frames = 0;
        }
        gcc_phat_set_score_ema_alpha(
            s_sustained_frames > 70U && !analyze_instant ? 0.15f : 0.33f);
        bool strong_snap = doa_event_is_strong_snap(
            rms, transient_rms, transient_peak_ratio,
            doa_result.instant_score);
        bool mirror_ambiguous =
            doa_result.instant_mirror_ratio >= TRANSIENT_MIRROR_RATIO_MAX;
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
                             doa_event_can_lock_single_frame(
                                 s_doa_tracker.initialized, brms, s_rms_floor) &&
                             (doa_event_use_fast_path(
                                  rms, transient_rms, transient_peak_ratio,
                                  doa_result.instant_score, doa_result.instant_confidence) ||
                              nf_transient);
        if (transient_doa && !nf_transient) {
            angle_raw = direction_calibrate_doa(doa_result.instant_angle_deg);
        }
        if (transient_doa) {
            /* A coherent one-frame onset need not wait for the RMS EMA to
             * rise. Its score, confidence and mirror tests already qualified it. */
            s_source_active = source_active = true;
            s_source_hold_frames = 6U;
            s_source_idle_frames = 0;
        }
        const char *event_tag = strong_snap ? " snap"
            : (nf_transient ? " nfsnap" : (transient_doa ? " tap" : ""));
        float tracker_confidence = transient_doa
            ? doa_result.instant_confidence : doa_result.confidence;
        if (nf_transient) {
            /* onset 拟合质量已由残差门限把关, 置信高于对侧镜像门限,
             * 否则 20cm 响指换新方向会被对侧保护拦成"不亮灯" */
            tracker_confidence = NF_SNAP_CONFIDENCE;
        }
        float track_min_energy = fmaxf(DOA_TRACK_MIN_ENERGY_ABS,
                                       s_rms_floor * DOA_TRACK_MIN_ENERGY_RATIO);
        bool srp_quality_ok = srp_valid &&
                              !chain_clipped &&
                              /* Broad low-frequency lobes can exceed .60
                               * at 180 degrees without a competing peak. */
                              doa_result.mirror_ratio < 0.90f &&
                              (doa_result.mirror_ratio < TRANSIENT_MIRROR_RATIO_MAX ||
                               doa_result.confidence >= 0.16f) &&
                              doa_result.score >= DOA_TRACK_MIN_SCORE;
        bool localization_valid = source_active &&
                                  ((srp_quality_ok && (brms >= track_min_energy || weak_evidence) &&
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
        /* A confirmed transient may have only one frame to change the lamp. */
        if (tracker_updated && transient_doa) doa_display_gate_init(&s_doa_display_gate);
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
        /* 灯效读取已认可的连续跟踪角，避免先量化成22.5°灯位、再动画导致
         * 转圈卡顿；只更新此显示副本，不改变s_doa_angle/网页display_angle。 */
        if (tracker_updated && display_active && isfinite(angle)) s_effect_direction = angle;
        led_effect_scene_t led_scene;
        doa_led_scene_update(&s_doa_led_scene, &doa_result,
            source_active && !chain_clipped && brms >= fmaxf(.0004f, s_rms_floor * 1.8f),
            s_led_lit, s_effect_direction,
            brms, s_rms_floor, (uint32_t)(esp_timer_get_time() / 1000), &led_scene);
        led_submit_scene(&led_scene, (uint32_t)(esp_timer_get_time() / 1000));
#endif

        frame_idx++;
#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
        if (source_active && !last_source_active) onset_log_frames = 6U;
        if ((frame_idx % 16U) == 0U || transient_doa || strong_snap ||
            onset_log_frames || source_active != last_source_active ||
            (transient_peak_ratio >= 1.6f && transient_rms >= .0045f)) {
            doa_diagnostic_t d = {
                .angle = angle, .raw = angle_raw, .rms = {rms0,rms1,rms2,rms3},
                .transient = transient_rms, .peak_ratio = transient_peak_ratio,
                .score = doa_result.score, .second = doa_result.second_score,
                .confidence = doa_result.confidence, .mirror = doa_result.mirror_ratio,
                .band_rms = brms, .floor = s_rms_floor,
                .instant_angle = direction_calibrate_doa(doa_result.instant_angle_deg),
                .instant_score = doa_result.instant_score,
                .instant_confidence = doa_result.instant_confidence,
                .instant_mirror = doa_result.instant_mirror_ratio,
                .active = source_active, .display = display_active, .updated = tracker_updated,
                .clips = clip_count, .pixel = s_doa_display_gate.held_pixel,
                .processing_us = (long)(esp_timer_get_time() - frame_start_us),
                .overflows = s_rx_overflows, .event = event_tag,
                .frame_ms = (uint32_t)(frame_start_us / 1000),
                .sustained_frames = s_sustained_frames,
                .instant_analysis = analyze_instant, .led_lit = s_led_lit,
                .spatial_peaks = doa_result.display_peak_count, .visible_sources = led_scene.count,
                .mic_mask = doa_result.active_mic_mask, .pairs = doa_result.active_pair_count,
            };
            xQueueSend(s_doa_log_queue, &d, 0);
        }
        if (onset_log_frames) onset_log_frames--;
        last_source_active = source_active;
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

        /* Blocking I2S reads normally let IDLE run. If processing overruns,
         * yield at most every 250 ms: at 100 Hz tick rate, sleeping 10 ms on
         * every 21 ms frame would itself cause continuous DMA queue loss. */
        TickType_t yield_now = xTaskGetTickCount();
        if ((esp_timer_get_time() - frame_start_us) >= 21000LL &&
            yield_now - last_forced_yield >= pdMS_TO_TICKS(250)) {
            vTaskDelay(1);
            last_forced_yield = yield_now;
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
    ESP_LOGI(TAG, " Array r=%.1fmm, Fs=%d Hz", (double)MIC_ARRAY_RADIUS_MM, SAMPLE_RATE);
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

#ifdef DOA_PCM_STREAM
    if (es7210_ret == ESP_OK) doa_pcm_stream_start(s_i2s_rx, SAMPLE_RATE, &s_rx_overflows);
    return;
#endif

    /* 算法与模型初始化 (按模式) */
#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
    gcc_phat_init(FRAME_LEN);
    doa_tracker_init(&s_doa_tracker, DOA_TRACK_MIN_CONF);
    s_doa_tracker.acquisition_frames = 5U;
    doa_display_gate_init(&s_doa_display_gate);
    doa_led_scene_init(&s_doa_led_scene);
    led_start_effects();
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
#if APP_MODE == APP_MODE_SOUND_LOCALIZATION
        s_doa_log_queue = xQueueCreate(16, sizeof(doa_diagnostic_t));
        configASSERT(s_doa_log_queue);
        configASSERT(xTaskCreatePinnedToCore(doa_log_task, "doa_log", 4096,
                                            NULL, 2, NULL, 0) == pdPASS);
#endif
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
