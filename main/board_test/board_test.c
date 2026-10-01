/*
 * 板级自检 (Board Self-Test) 实现
 *
 * 测试项:
 *   1. Wi-Fi 连接 (用于上传, 顺带验证 WiFi)
 *   2. 两个 I2C 引出排针 (i2c1 / i2c2) 总线扫描
 *   3. 两个 ADC (adc_3v3 / adc5v) 电压读取
 *   4. RGB 灯环 (WS2812) 点亮一圈 1s 后熄灭
 *   5. 无源蜂鸣器 (GPIO4) 响几声
 *   6. 两个 UART 排针 + 两个 IO 排针: 持续高低电平切换输出
 *   7. 麦克风阵列 (ES7210): 录制 5s 音频, 逐通道分析电平,
 *      生成 WAV 并上传到本地 PC 服务器 (顺带验证麦克风与 ES7210)
 */

#include "board_test.h"

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s_tdm.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_http_client.h"
#include "led_strip.h"

#include "es7210.h"
#include "wifi_server.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "sys/select.h"

static const char *TAG = "board_test";

/* ================================================================ */
/*  I2S / ES7210 引脚 (与主程序一致)                                  */
/* ================================================================ */
#define TEST_I2S_MCK_IO   (16)
#define TEST_I2S_BCK_IO   (18)
#define TEST_I2S_WS_IO    (17)
#define TEST_I2S_DI_IO    (15)
#define TEST_ES7210_SDA   (8)
#define TEST_ES7210_SCL   (9)
#define TEST_ES7210_ADDR  (0x40)

static i2s_chan_handle_t s_i2s_rx = NULL;
static es7210_dev_handle_t s_es7210 = NULL;

/* 测试结果统计 */
static int s_pass = 0;
static int s_fail = 0;

/* ================================================================ */
/*  工具函数                                                         */
/* ================================================================ */
static void test_report(const char *item, bool ok, const char *detail)
{
    if (ok) {
        s_pass++;
        ESP_LOGI(TAG, "[PASS] %-20s | %s", item, detail ? detail : "");
    } else {
        s_fail++;
        ESP_LOGE(TAG, "[FAIL] %-20s | %s", item, detail ? detail : "");
    }
}

/* ================================================================ */
/*  1. I2C 总线扫描 (两个引出排针)                                    */
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
        ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

static bool i2c_probe_addr_stable(i2c_port_t port, uint8_t addr, int tries)
{
    int ack_count = 0;
    for (int i = 0; i < tries; i++) {
        if (i2c_probe_addr(port, addr) == ESP_OK) {
            ack_count++;
        }
    }
    return (ack_count == tries);
}

/* 检查 I2C 总线空闲状态: 返回 true 表示 SDA/SCL 均为高 (总线空闲) */
static bool i2c_bus_idle(int sda, int scl)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    vTaskDelay(pdMS_TO_TICKS(10));
    int sda_lvl = gpio_get_level(sda);
    int scl_lvl = gpio_get_level(scl);
    ESP_LOGI(TAG, "I2C空闲: SDA=%d SCL=%d", sda_lvl, scl_lvl);
    return (sda_lvl == 1 && scl_lvl == 1);
}

static void scan_i2c_bus(i2c_port_t port, const char *name, int sda, int scl)
{
    bool idle = i2c_bus_idle(sda, scl);
    if (!idle) {
        ESP_LOGI(TAG, "%s: 总线被拉低, 无I2C设备", name);
        test_report(name, false, "总线被拉低");
        gpio_reset_pin(sda);
        gpio_reset_pin(scl);
        return;
    }

    gpio_reset_pin(sda);
    gpio_reset_pin(scl);

    i2c_config_t cfg = {
        .sda_io_num = sda,
        .scl_io_num = scl,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    esp_err_t ret = i2c_param_config(port, &cfg);
    if (ret == ESP_OK) {
        ret = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    }
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "%s: I2C驱动安装失败, 无I2C设备", name);
        test_report(name, false, "I2C驱动安装失败");
        return;
    }

    char detail[512] = "";
    int found = 0;
    for (uint8_t addr = 0x00; addr <= 0x7F; addr++) {
        if (i2c_probe_addr(port, addr) == ESP_OK) {
            char tmp[8];
            snprintf(tmp, sizeof(tmp), "%s0x%02X", found ? "," : "", addr);
            strncat(detail, tmp, sizeof(detail) - strlen(detail) - 1);
            found++;
        }
    }
    i2c_driver_delete(port);
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);

    if (found > 0) {
        ESP_LOGI(TAG, "%s: %s", name, detail);
        test_report(name, true, detail);
    } else {
        ESP_LOGI(TAG, "%s: 无I2C设备", name);
        test_report(name, false, "无I2C设备");
    }
}

/* ================================================================ */
/*  3. ADC 电压读取: 每秒循环输出两个 ADC 电压                        */
/* ================================================================ */
/* 通道定义: adc_3v3=GPIO6/ADC1_CH5, adc5v=GPIO5/ADC1_CH4 */
typedef struct {
    const char *name;
    int gpio;
    adc_channel_t chan;
} adc_chan_desc_t;

static const adc_chan_desc_t s_adc_chans[] = {
    { "ADC 3V3 (GPIO6)", TEST_ADC_3V3_IO, ADC_CHANNEL_5 },
    { "ADC 5V  (GPIO5)", TEST_ADC_5V_IO,  ADC_CHANNEL_4 },
};
#define ADC_CHAN_COUNT  (sizeof(s_adc_chans) / sizeof(s_adc_chans[0]))

static void adc_monitor_task(void *arg)
{
    adc_oneshot_unit_handle_t adc = NULL;
    adc_cali_handle_t cali[ADC_CHAN_COUNT] = { NULL };

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&unit_cfg, &adc) != ESP_OK) {
        test_report("ADC 监控", false, "ADC 单元创建失败");
        vTaskDelete(NULL);
        return;
    }
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    for (int i = 0; i < ADC_CHAN_COUNT; i++) {
        if (adc_oneshot_config_channel(adc, s_adc_chans[i].chan, &chan_cfg) != ESP_OK) {
            ESP_LOGE(TAG, "%s 通道配置失败", s_adc_chans[i].name);
        }
        /* 校准 (ESP32-S3 仅支持 curve fitting 方案) */
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_1,
            .chan = s_adc_chans[i].chan,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali[i]) != ESP_OK) {
            cali[i] = NULL;
        }
    }

    ESP_LOGI(TAG, "ADC 监控启动: 每秒输出一次电压");
    while (1) {
        for (int i = 0; i < ADC_CHAN_COUNT; i++) {
            int raw = 0;
            int mv = 0;
            bool ok = (adc_oneshot_read(adc, s_adc_chans[i].chan, &raw) == ESP_OK);
            if (ok && cali[i]) {
                adc_cali_raw_to_voltage(cali[i], raw, &mv);
            }
            if (ok) {
                ESP_LOGI(TAG, "  %s: raw=%d  %d.%03dV", s_adc_chans[i].name,
                         raw, mv / 1000, mv % 1000);
            } else {
                ESP_LOGE(TAG, "  %s: 读取失败", s_adc_chans[i].name);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));   /* 每秒输出一次 */
    }

    /* 不会执行到 (无限循环) */
    adc_oneshot_del_unit(adc);
    vTaskDelete(NULL);
}

static void start_adc_monitor(void)
{
    xTaskCreatePinnedToCore(adc_monitor_task, "adc_monitor",
                            4096, NULL, 4, NULL, 0);
    test_report("ADC 监控", true, "每秒输出 3V3/5V 电压");
}

/* ================================================================ */
/*  4. RGB 灯环测试: 点亮一圈 1s 后熄灭                               */
/* ================================================================ */
static void test_rgb_ring(void)
{
    gpio_reset_pin(TEST_WS2812_IO);
    led_strip_handle_t strip = NULL;
    led_strip_config_t cfg = {
        .strip_gpio_num = TEST_WS2812_IO,
        .max_leds = TEST_LED_COUNT,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 0,
        .flags.with_dma = false,
    };
    esp_err_t err = led_strip_new_rmt_device(&cfg, &rmt, &strip);
    if (err != ESP_OK) {
        test_report("RGB 灯环", false, "WS2812 初始化失败");
        return;
    }
    led_strip_clear(strip);
    led_strip_refresh(strip);

    /* 依次点亮每个灯 (红→绿→蓝 依次循环), 最后全部点亮形成一圈 */
    /* 颜色数组: 因新灯珠为RGB顺序而驱动为GRB格式, 交换R/G值以正确显色 */
    const uint8_t colors[3][3] = {{0, 127, 0}, {127, 0, 0}, {0, 0, 127}};
    for (int i = 0; i < TEST_LED_COUNT; i++) {
        uint8_t r = colors[i % 3][0], g = colors[i % 3][1], b = colors[i % 3][2];
        led_strip_set_pixel(strip, i, r, g, b);
        led_strip_refresh(strip);
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    /* 全亮 5s (绿色) */
    for (int i = 0; i < TEST_LED_COUNT; i++) {
        led_strip_set_pixel(strip, i, 127, 0, 0);
    }
    led_strip_refresh(strip);
    vTaskDelay(pdMS_TO_TICKS(5000));
    led_strip_clear(strip);
    led_strip_refresh(strip);
    led_strip_del(strip);

    test_report("RGB 灯环", true, "一圈点亮 5s 后熄灭");
}

/* ================================================================ */
/*  5. 蜂鸣器测试: 响几声                                             */
/* ================================================================ */
static void test_buzzer(void)
{
    /* 无源蜂鸣器: 用 4kHz PWM 驱动几下 */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << TEST_BUZZER_IO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t ch = {
        .gpio_num = TEST_BUZZER_IO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_timer_config(&timer);
    ledc_channel_config(&ch);

    for (int i = 0; i < 3; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(200));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    gpio_set_level(TEST_BUZZER_IO, 1); /* 高电平关闭 NPN 驱动 */

    test_report("蜂鸣器", true, "响 3 下 (2kHz PWM)");
}

/* ================================================================ */
/*  6. UART 回环测试 + IO 排针 toggle (后台任务)                      */
/*     UART1: TX=GPIO12, RX=GPIO13;  UART2: TX=GPIO47, RX=GPIO48    */
/*     交叉连接: UART1.TX(12)->UART2.RX(48), UART1.RX(13)<-UART2.TX(47) */
/*     交替: UART1 发 "hello"→UART2 收, UART2 发 "world"→UART1 收      */
/* ================================================================ */
/* IO 排针 (普通 GPIO 输出, 持续高低切换; 47/48 已用作 UART2) */
static const gpio_num_t s_io_pins[] = {
    46, 1, 2,   /* IO1 (47/48 已用作 UART2) */
    42, 41, 40, 39, 38, /* IO2 */
};
#define IO_PIN_COUNT  (sizeof(s_io_pins) / sizeof(s_io_pins[0]))

static void uart_loopback_task(void *arg)
{
    /* --- UART1: TX=GPIO12, RX=GPIO13 --- */
    uart_config_t u1 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_param_config(UART_NUM_1, &u1) != ESP_OK ||
        uart_set_pin(UART_NUM_1, TEST_UART1_TX, TEST_UART1_RX, -1, -1) != ESP_OK ||
        uart_driver_install(UART_NUM_1, 1024, 1024, 0, NULL, 0) != ESP_OK) {
        test_report("UART1", false, "UART1 初始化失败");
    } else {
        test_report("UART1", true, "GPIO12(TX)/13(RX) 初始化成功");
    }

    /* --- UART2: TX=GPIO47, RX=GPIO48 --- */
    uart_config_t u2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_param_config(UART_NUM_2, &u2) != ESP_OK ||
        uart_set_pin(UART_NUM_2, TEST_UART2_TX, TEST_UART2_RX, -1, -1) != ESP_OK ||
        uart_driver_install(UART_NUM_2, 1024, 1024, 0, NULL, 0) != ESP_OK) {
        test_report("UART2", false, "UART2 初始化失败");
    } else {
        test_report("UART2", true, "GPIO47(TX)/48(RX) 初始化成功");
    }

    /* --- IO 排针配置为输出 --- */
    bool io_ok[IO_PIN_COUNT] = {false};
    for (int i = 0; i < IO_PIN_COUNT; i++) {
        if (s_io_pins[i] < 0 || s_io_pins[i] >= GPIO_NUM_MAX) {
            ESP_LOGE(TAG, "非法 GPIO %d, 跳过", s_io_pins[i]);
            continue;
        }
        gpio_config_t one = {
            .pin_bit_mask = (1ULL << s_io_pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&one) == ESP_OK) {
            io_ok[i] = true;
        } else {
            ESP_LOGW(TAG, "GPIO%d 配置失败, 跳过", s_io_pins[i]);
        }
    }

    ESP_LOGI(TAG, "UART 回环测试启动: 交替 UART1→UART2 / UART2→UART1");
    ESP_LOGI(TAG, "  (请交叉连接: GPIO12<->GPIO48, GPIO13<->GPIO47)");

    const char *msg_u1 = "hello\r\n";
    const char *msg_u2 = "world\r\n";
    uint8_t rx_buf[64];

    bool level = true;
    int loop = 0;
    while (1) {
        if (loop % 2 == 0) {
            uart_write_bytes(UART_NUM_1, msg_u1, strlen(msg_u1));
            int len = uart_read_bytes(UART_NUM_2, rx_buf, sizeof(rx_buf) - 1,
                                      pdMS_TO_TICKS(100));
            if (len > 0) {
                rx_buf[len] = '\0';
                ESP_LOGI(TAG, "UART1→UART2: UART2 收到 %d 字节: \"%s\"", len, (char *)rx_buf);
            } else {
                ESP_LOGW(TAG, "UART1→UART2: UART2 未收到数据");
            }
        } else {
            uart_write_bytes(UART_NUM_2, msg_u2, strlen(msg_u2));
            int len = uart_read_bytes(UART_NUM_1, rx_buf, sizeof(rx_buf) - 1,
                                      pdMS_TO_TICKS(100));
            if (len > 0) {
                rx_buf[len] = '\0';
                ESP_LOGI(TAG, "UART2→UART1: UART1 收到 %d 字节: \"%s\"", len, (char *)rx_buf);
            } else {
                ESP_LOGW(TAG, "UART2→UART1: UART1 未收到数据");
            }
        }

        for (int i = 0; i < IO_PIN_COUNT; i++) {
            if (io_ok[i]) {
                gpio_set_level(s_io_pins[i], level);
            }
        }
        level = !level;

        loop++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void start_uart_loopback(void)
{
    xTaskCreatePinnedToCore(uart_loopback_task, "uart_loopback",
                            8192, NULL, 4, NULL, 0);
    test_report("UART 回环", true,
                "交替: UART1(12/13)→UART2(47/48) / UART2→UART1");
}

/* ================================================================ */
/*  7. 麦克风阵列: 录制 5s, 分析电平, 生成 WAV 并上传                   */
/* ================================================================ */
static void init_i2s_es7210(void)
{
    i2s_chan_config_t rx_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_cfg, NULL, &s_i2s_rx));

    i2s_tdm_config_t tdm_cfg = {
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO,
                        I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
        .clk_cfg = {
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .sample_rate_hz = TEST_SAMPLE_RATE,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .gpio_cfg = {
            .mclk = TEST_I2S_MCK_IO,
            .bclk = TEST_I2S_BCK_IO,
            .ws   = TEST_I2S_WS_IO,
            .dout = -1,
            .din  = TEST_I2S_DI_IO,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(s_i2s_rx, &tdm_cfg));

    /* I2C 驱动 (ES7210) */
    i2c_config_t i2c_cfg = {
        .sda_io_num = TEST_ES7210_SDA,
        .scl_io_num = TEST_ES7210_SCL,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    uint8_t addr = TEST_ES7210_ADDR;
    bool found = false;
    for (uint8_t a = 0x40; a <= 0x43; a++) {
        if (i2c_probe_addr(I2C_NUM_0, a) == ESP_OK) {
            addr = a;
            found = true;
            break;
        }
    }
    if (!found) {
        ESP_LOGE(TAG, "ES7210 未应答 (0x40~0x43)");
        test_report("ES7210", false, "I2C 无应答");
        return;
    }

    es7210_i2c_config_t es_i2c = { .i2c_port = I2C_NUM_0, .i2c_addr = addr };
    if (es7210_new_codec(&es_i2c, &s_es7210) != ESP_OK) {
        test_report("ES7210", false, "创建句柄失败");
        return;
    }
    es7210_codec_config_t codec_cfg = {
        .i2s_format = ES7210_I2S_FMT_I2S,
        .mclk_ratio = I2S_MCLK_MULTIPLE_256,
        .sample_rate_hz = TEST_SAMPLE_RATE,
        .bit_width = ES7210_I2S_BITS_16B,
        .mic_bias = ES7210_MIC_BIAS_2V87,
        .mic_gain = ES7210_MIC_GAIN_37_5DB,   /* 最大模拟增益 */
        .flags.tdm_enable = true,
    };
    esp_err_t ret = es7210_config_codec(s_es7210, &codec_cfg);
    if (ret == ESP_OK) {
        ret = es7210_config_volume(s_es7210, 24);   /* +24dB 数字增益 */
    }
    if (ret != ESP_OK) {
        test_report("ES7210", false, "配置失败");
        es7210_del_codec(s_es7210);
        s_es7210 = NULL;
        return;
    }
    test_report("ES7210", true, "I2C 配置成功");
}

/* 写入 WAV 头 (16-bit PCM, 多通道交织) */
static void build_wav_header(uint8_t *hdr, int sample_rate, int channels,
                             int bits, uint32_t data_bytes)
{
    uint32_t byte_rate = (uint32_t)sample_rate * channels * bits / 8;
    uint16_t block_align = (uint16_t)(channels * bits / 8);
    uint32_t total = data_bytes + 36;

    memcpy(hdr, "RIFF", 4);
    hdr[4] = total & 0xff; hdr[5] = (total >> 8) & 0xff;
    hdr[6] = (total >> 16) & 0xff; hdr[7] = (total >> 24) & 0xff;
    memcpy(hdr + 8, "WAVEfmt ", 8);
    hdr[16] = 16; hdr[17] = 0; hdr[18] = 0; hdr[19] = 0;   /* fmt chunk size */
    hdr[20] = 1; hdr[21] = 0;                               /* PCM */
    hdr[22] = channels & 0xff; hdr[23] = (channels >> 8) & 0xff;
    hdr[24] = sample_rate & 0xff; hdr[25] = (sample_rate >> 8) & 0xff;
    hdr[26] = (sample_rate >> 16) & 0xff; hdr[27] = (sample_rate >> 24) & 0xff;
    hdr[28] = byte_rate & 0xff; hdr[29] = (byte_rate >> 8) & 0xff;
    hdr[30] = (byte_rate >> 16) & 0xff; hdr[31] = (byte_rate >> 24) & 0xff;
    hdr[32] = block_align & 0xff; hdr[33] = (block_align >> 8) & 0xff;
    hdr[34] = bits & 0xff; hdr[35] = (bits >> 8) & 0xff;
    memcpy(hdr + 36, "data", 4);
    hdr[40] = data_bytes & 0xff; hdr[41] = (data_bytes >> 8) & 0xff;
    hdr[42] = (data_bytes >> 16) & 0xff; hdr[43] = (data_bytes >> 24) & 0xff;
}

static bool probe_http_server(const char *ip, int port)
{
    int sock = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return false;

    /* 设置非阻塞模式，用 select 实现精确超时的 connect */
    int flags = lwip_fcntl(sock, F_GETFL, 0);
    lwip_fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = inet_addr(ip),
    };

    int ret = lwip_connect(sock, (struct sockaddr *)&dest, sizeof(dest));
    if (ret < 0 && errno != EINPROGRESS) {
        lwip_close(sock);
        return false;
    }
    if (ret == 0) {
        /* 连接立即完成（回环地址等） */
        goto connected;
    }

    /* select 等待连接完成，200ms 超时 */
    fd_set wset;
    FD_ZERO(&wset);
    FD_SET(sock, &wset);
    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    ret = select(sock + 1, NULL, &wset, NULL, &tv);
    if (ret <= 0 || !FD_ISSET(sock, &wset)) {
        lwip_close(sock);
        return false;
    }

    /* 检查连接是否被拒绝（RST） */
    int err = 0;
    socklen_t errlen = sizeof(err);
    if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &errlen) < 0 || err != 0) {
        lwip_close(sock);
        return false;
    }

connected:
    /* 恢复阻塞模式用于后续读写 */
    lwip_fcntl(sock, F_SETFL, flags);

    /* 只对 recv 设 1s 超时 */
    struct timeval tv_recv = { .tv_sec = 1, .tv_usec = 0 };
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv_recv, sizeof(tv_recv));

    const char *req = "GET / HTTP/1.0\r\n\r\n";
    if (lwip_write(sock, req, strlen(req)) < 0) {
        lwip_close(sock);
        return false;
    }

    char resp[64] = {0};
    int n = lwip_read(sock, resp, sizeof(resp) - 1);
    lwip_close(sock);

    return (n > 0 && strstr(resp, "200") != NULL);
}

static bool discover_server_ip(char *out_ip, size_t out_size)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        ESP_LOGE(TAG, "获取网络接口失败");
        return false;
    }

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        ESP_LOGE(TAG, "获取本机IP失败");
        return false;
    }

    uint32_t my_ip = ntohl(ip_info.ip.addr);
    uint32_t gw = ntohl(ip_info.gw.addr);
    uint8_t a = (my_ip >> 24) & 0xff;
    uint8_t b = (my_ip >> 16) & 0xff;
    uint8_t c = (my_ip >> 8) & 0xff;
    uint8_t my_last = my_ip & 0xff;
    uint8_t gw_last = gw & 0xff;

    ESP_LOGI(TAG, "本机IP: %u.%u.%u.%u, 扫描同网段HTTP端口%d ...",
             a, b, c, my_last, TEST_SERVER_PORT);

    for (int i = 1; i <= TEST_SERVER_SCAN_MAX; i++) {
        if (i == my_last || i == gw_last) continue;
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), "%u.%u.%u.%u", a, b, c, i);

        if (probe_http_server(ip_str, TEST_SERVER_PORT)) {
            snprintf(out_ip, out_size, "%s", ip_str);
            ESP_LOGI(TAG, "发现上传服务器: %s:%d", ip_str, TEST_SERVER_PORT);
            return true;
        }
    }

    ESP_LOGE(TAG, "未发现上传服务器 (网段 %u.%u.%u.x:%d)", a, b, c, TEST_SERVER_PORT);
    return false;
}

static void record_and_upload(void)
{
    if (s_es7210 == NULL) {
        ESP_LOGE(TAG, "ES7210 不可用, 跳过录音上传");
        return;
    }

    char server_ip[16] = "";
    if (!discover_server_ip(server_ip, sizeof(server_ip))) {
        ESP_LOGE(TAG, "未找到上传服务器, 跳过上传");
    }

    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_rx));

    /* 麦克风方向映射 (与主程序一致): ch0=270°, ch1=180°, ch2=0°, ch3=90° */
    static const int mic_deg[TEST_CHAN_NUM] = {270, 180, 0, 90};

    const uint32_t frames_per_mic = TEST_SAMPLE_RATE * TEST_RECORD_SECONDS;
    const size_t frame_bytes = TEST_CHAN_NUM * sizeof(int16_t);
    /* 静态缓冲, 避免占用任务栈 */
    static int16_t buf[TEST_CHAN_NUM * TEST_I2S_BUF_FRAMES];
    static int16_t mono[TEST_I2S_BUF_FRAMES];

    /* 逐通道 RMS 累加 */
    double rms_sq[TEST_CHAN_NUM] = {0};
    uint64_t rms_count[TEST_CHAN_NUM] = {0};
    int16_t peak[TEST_CHAN_NUM] = {0};
    bool upload_ok[TEST_CHAN_NUM] = {false};

    ESP_LOGI(TAG, "开始逐麦克风录制: 每个麦克风 %d 秒, 共 %d 段 (合计 %d 秒)",
             TEST_RECORD_SECONDS, TEST_CHAN_NUM,
             TEST_RECORD_SECONDS * TEST_CHAN_NUM);

    for (int ch = 0; ch < TEST_CHAN_NUM; ch++) {
        char fname[32];
        snprintf(fname, sizeof(fname), "mic%d_ch%d_%ddeg.wav", ch + 1, ch, mic_deg[ch]);

        bool has_server = (server_ip[0] != '\0');
        esp_http_client_handle_t client = NULL;

        if (has_server) {
            char url[128];
            snprintf(url, sizeof(url), "http://%s:%d%s",
                     server_ip, TEST_SERVER_PORT, TEST_UPLOAD_PATH);
            ESP_LOGI(TAG, "录制 MIC%d (@%d°) %d 秒并上传 (文件名 %s) ...",
                     ch + 1, mic_deg[ch], TEST_RECORD_SECONDS, fname);

            esp_http_client_config_t http_cfg = {
                .url = url,
                .method = HTTP_METHOD_POST,
                .timeout_ms = 30000,
                .buffer_size = 4096,
            };
            client = esp_http_client_init(&http_cfg);
            if (client == NULL) {
                ESP_LOGE(TAG, "MIC%d: HTTP 客户端创建失败", ch + 1);
                has_server = false;
            } else {
                esp_http_client_set_header(client, "X-Filename", fname);
            }
        } else {
            ESP_LOGI(TAG, "录制 MIC%d (@%d°) %d 秒 (无服务器, 仅本地分析) ...",
                     ch + 1, mic_deg[ch], TEST_RECORD_SECONDS);
        }

        if (has_server) {
            const uint32_t data_bytes = frames_per_mic * sizeof(int16_t);
            esp_err_t err = esp_http_client_open(client, (int)(data_bytes + 44));
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "MIC%d: 连接服务器失败: %s",
                         ch + 1, esp_err_to_name(err));
                esp_http_client_cleanup(client);
                client = NULL;
                has_server = false;
            } else {
                uint8_t wav_hdr[44];
                build_wav_header(wav_hdr, TEST_SAMPLE_RATE, 1, 16, data_bytes);
                esp_http_client_write(client, (char *)wav_hdr, 44);
            }
        }

        /* 流式录制, 仅提取当前通道 ch 的样本 */
        uint32_t frames_done = 0;
        while (frames_done < frames_per_mic) {
            size_t bytes_read = 0;
            esp_err_t r = i2s_channel_read(s_i2s_rx, buf, sizeof(buf),
                                           &bytes_read, pdMS_TO_TICKS(500));
            if (r != ESP_OK || bytes_read == 0) {
                continue;
            }
            int frames = bytes_read / frame_bytes;
            /* 截断到最后所需帧数, 避免多录 */
            if (frames_done + frames > frames_per_mic) {
                frames = frames_per_mic - frames_done;
            }
            for (int i = 0; i < frames; i++) {
                int16_t v = buf[i * TEST_CHAN_NUM + ch];
                mono[i] = v;
                rms_sq[ch] += (double)v * (double)v;
                rms_count[ch]++;
                if (v > peak[ch]) peak[ch] = v;
            }
            if (has_server) {
                esp_http_client_write(client, (char *)mono,
                                      frames * sizeof(int16_t));
            }
            frames_done += frames;
        }

        if (client) {
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
        }
        upload_ok[ch] = has_server;
        ESP_LOGI(TAG, "MIC%d (@%d°) 录制完成 (%d 帧)%s",
                 ch + 1, mic_deg[ch], frames_done,
                 has_server ? " 已上传" : "");
    }

    ESP_ERROR_CHECK(i2s_channel_disable(s_i2s_rx));

    /* 打印逐通道结果 */
    float rms[TEST_CHAN_NUM];
    int dominant = 0;
    float dom_val = -1.0f;
    for (int ch = 0; ch < TEST_CHAN_NUM; ch++) {
        rms[ch] = (rms_count[ch] > 0)
            ? (float)sqrt(rms_sq[ch] / (double)rms_count[ch]) / 32768.0f
            : 0.0f;
        if (rms[ch] > dom_val) {
            dom_val = rms[ch];
            dominant = ch;
        }
        ESP_LOGI(TAG, "MIC%d (@%d°): RMS=%.4f  peak=%d  upload=%s",
                 ch + 1, mic_deg[ch], rms[ch], peak[ch],
                 upload_ok[ch] ? "OK" : "FAIL");
    }

    int ok_cnt = 0;
    for (int ch = 0; ch < TEST_CHAN_NUM; ch++) {
        if (upload_ok[ch]) ok_cnt++;
    }
    char detail[96];
    snprintf(detail, sizeof(detail),
             "%d/%d 段上传成功, 最强 MIC%d @%d° (RMS=%.4f)",
             ok_cnt, TEST_CHAN_NUM, dominant + 1, mic_deg[dominant], dom_val);
    test_report("麦克风录音上传", ok_cnt == TEST_CHAN_NUM, detail);

    /* 检查每个麦克风是否都有信号 */
    for (int ch = 0; ch < TEST_CHAN_NUM; ch++) {
        char d[48];
        snprintf(d, sizeof(d), "MIC%d @%d° RMS=%.4f", ch + 1, mic_deg[ch], rms[ch]);
        test_report("麦克风", rms[ch] > 0.0005f, d);
    }
}

/* ================================================================ */
/*  主入口                                                           */
/* ================================================================ */
static void board_test_run(void);   /* 前向声明 (定义在下方) */

static void board_test_task(void *arg)
{
    board_test_run();
    ESP_LOGI(TAG, "自检流程结束, 任务退出");
    vTaskDelete(NULL);
}

void board_test_start(void)
{
    xTaskCreatePinnedToCore(board_test_task, "board_test",
                            BOARD_TEST_TASK_STACK, NULL,
                            BOARD_TEST_TASK_PRIO, NULL, 0);
}

void board_test_run(void)
{
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  板级自检模式 (Board Self-Test)");
    ESP_LOGI(TAG, "======================================");

    /* 1. Wi-Fi 连接 (用于上传, 顺带验证 WiFi) */
    ESP_LOGI(TAG, "[1/7] Wi-Fi 连接 ...");
    wifi_server_init();
    test_report("Wi-Fi", true, "连接成功");

    /* 2. 两个 I2C 排针扫描 */
    ESP_LOGI(TAG, "[2/7] I2C 总线扫描 ...");
    scan_i2c_bus(I2C_NUM_0, "I2C1 (GPIO11/10)", TEST_I2C1_SDA, TEST_I2C1_SCL);
    scan_i2c_bus(I2C_NUM_1, "I2C2 (GPIO14/7)", TEST_I2C2_SDA, TEST_I2C2_SCL);

    /* 2.5 对比验证: 用与 ES7210 相同的 I2C 总线 (GPIO8/9, I2C_NUM_0) 扫描.
     * 用于验证设备本身是否正常: 把怀疑有问题的设备临时接到 GPIO8/9,
     * 若这里能扫到, 说明设备正常, 问题在排针接线/原理图引脚; 若这里也扫不到, 说明设备或接线有问题. */
    ESP_LOGI(TAG, "[2.5/7] 对比扫描 ES7210 总线 (GPIO8/9) ...");
    scan_i2c_bus(I2C_NUM_0, "ES7210总线 (GPIO8/9)", TEST_ES7210_SDA, TEST_ES7210_SCL);

    /* 3. 两个 ADC: 后台任务每秒输出一次电压 (持续运行) */
    ESP_LOGI(TAG, "[3/7] ADC 电压监控 (每秒输出) ...");
    start_adc_monitor();

    /* 4. RGB 灯环 */
    ESP_LOGI(TAG, "[4/7] RGB 灯环测试 ...");
    test_rgb_ring();

    /* 5. 蜂鸣器 */
    ESP_LOGI(TAG, "[5/7] 蜂鸣器测试 ...");
    test_buzzer();

    /* 6. 麦克风录制 + 上传 (每个麦克风 5s, 共 4 段) */
    ESP_LOGI(TAG, "[6/7] 麦克风录制 + 上传 ...");
    init_i2s_es7210();
    record_and_upload();

    /* 7. UART 回环测试 (放最后, 后台任务一直循环不停止) */
    ESP_LOGI(TAG, "[7/7] UART 回环测试 (持续循环) ...");
    start_uart_loopback();

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "自检完成: PASS=%d  FAIL=%d", s_pass, s_fail);
    ESP_LOGI(TAG, "======================================");
}