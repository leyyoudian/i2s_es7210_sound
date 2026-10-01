/*
 * 例程 5: ES7210 麦克风阵列录音
 *
 * 板载 4 颗 SPH1642HT5H MEMS 麦克风经 ES7210 音频 ADC
 * 以 I2S TDM 模式同步采集 4 通道音频:
 *   I2S: MCLK=GPIO16, BCLK=GPIO18, WS=GPIO17, DIN=GPIO15
 *   I2C: SDA=GPIO8,  SCL=GPIO9 (ES7210 寄存器配置)
 *
 * 本例程演示: 编解码器初始化 -> 4ch 采集 -> 逐通道实时电平分析,
 * 打印各通道 RMS, 直观验证麦克风方向性 (对准某支麦克风说话,
 * 对应通道电平明显更高).
 */

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2s_tdm.h"
#include "driver/i2c.h"
#include "es7210.h"
#include "examples.h"

static const char *TAG = "es7210_rec";

/* ---------------- 引脚 ---------------- */
#define I2S_MCK_IO   (16)
#define I2S_BCK_IO   (18)
#define I2S_WS_IO    (17)
#define I2S_DI_IO    (15)
#define I2C_SDA_IO   (8)
#define I2C_SCL_IO   (9)

/* ---------------- 音频参数 ---------------- */
#define SAMPLE_RATE  (48000)
#define CHAN_NUM     (4)
#define FRAME_MS     (100)                       /* 每 100ms 分析一次 */
#define FRAME_LEN    (SAMPLE_RATE / 1000 * FRAME_MS)
#define BUF_SAMPLES  (FRAME_LEN * CHAN_NUM)

#define ES7210_ADDR  (0x40)
#define ES7210_GAIN  (ES7210_MIC_GAIN_30DB)
#define ES7210_BIAS  (ES7210_MIC_BIAS_2V87)

static i2s_chan_handle_t s_i2s_rx = NULL;
static es7210_dev_handle_t s_es7210 = NULL;

static void init_i2s(void)
{
    i2s_chan_config_t rx_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_cfg, NULL, &s_i2s_rx));

    i2s_tdm_config_t tdm_cfg = {
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO,
                        I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
        .clk_cfg = {
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .sample_rate_hz = SAMPLE_RATE,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .gpio_cfg = {
            .mclk = I2S_MCK_IO,
            .bclk = I2S_BCK_IO,
            .ws   = I2S_WS_IO,
            .dout = -1,
            .din  = I2S_DI_IO,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(s_i2s_rx, &tdm_cfg));
}

static esp_err_t init_es7210(void)
{
    i2c_config_t i2c_cfg = {
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    es7210_i2c_config_t es_i2c = { .i2c_port = I2C_NUM_0, .i2c_addr = ES7210_ADDR };
    ESP_RETURN_ON_ERROR(es7210_new_codec(&es_i2c, &s_es7210), TAG, "es7210_new_codec");

    es7210_codec_config_t codec_cfg = {
        .i2s_format = ES7210_I2S_FMT_I2S,
        .mclk_ratio = I2S_MCLK_MULTIPLE_256,
        .sample_rate_hz = SAMPLE_RATE,
        .bit_width = ES7210_I2S_BITS_16B,
        .mic_bias = ES7210_BIAS,
        .mic_gain = ES7210_GAIN,
        .flags.tdm_enable = true,
    };
    return es7210_config_codec(s_es7210, &codec_cfg);
}

static float ch_rms(const int16_t *x, int n)
{
    double sum_sq = 0.0;
    for (int i = 0; i < n; i++) {
        float v = x[i] / 32768.0f;
        sum_sq += (double)v * v;
    }
    return (float)sqrt(sum_sq / n);
}

void example_es7210_start(void)
{
    ESP_LOGI(TAG, "=== ES7210 4-Mic 阵列录音例程 (%d Hz TDM) ===", SAMPLE_RATE);

    init_i2s();
    ESP_ERROR_CHECK(init_es7210());
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_rx));
    ESP_LOGI(TAG, "开始采集, 对着不同方向的麦克风说话观察电平变化");

    static int16_t buf[BUF_SAMPLES];
    while (1) {
        size_t bytes_read = 0;
        if (i2s_channel_read(s_i2s_rx, buf, sizeof(buf), &bytes_read,
                             pdMS_TO_TICKS(500)) != ESP_OK || bytes_read == 0) {
            continue;
        }
        int frames = bytes_read / (CHAN_NUM * sizeof(int16_t));

        float rms[CHAN_NUM];
        for (int ch = 0; ch < CHAN_NUM; ch++) {
            int16_t chbuf[FRAME_LEN];
            for (int i = 0; i < frames; i++) {
                chbuf[i] = buf[i * CHAN_NUM + ch];
            }
            rms[ch] = ch_rms(chbuf, frames);
        }
        printf("MIC1(270°)=%.4f  MIC2(180°)=%.4f  MIC3(0°)=%.4f  MIC4(90°)=%.4f\n",
               rms[0], rms[1], rms[2], rms[3]);
    }
}
