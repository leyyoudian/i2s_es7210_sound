/*
 * 板级自检 (Board Self-Test) 模块
 *
 * 用于生产/维修测试: 依次验证板上各外设是否正常, 并把麦克风录制结果
 * 上传到本地 PC 服务器 (单片机与 PC 处于同一网络).
 *
 * 切换方式: 在 i2s_es7210_record_example.c 中定义 ENABLE_BOARD_TEST 后,
 * app_main 将运行本自检流程而非正常检测流程.
 */

#ifndef BOARD_TEST_H
#define BOARD_TEST_H

#include <stdbool.h>

/* ================================================================ */
/*  测试模式开关                                                      */
/* ================================================================ */
/* 定义后 app_main 进入板级自检; 注释掉则恢复正常检测程序 */
//#define ENABLE_BOARD_TEST

/* ================================================================ */
/*  上传服务器配置 (PC 与单片机同一网络)                               */
/* ================================================================ */
#define TEST_SERVER_PORT    8080
#define TEST_UPLOAD_PATH    "/upload"
#define TEST_SERVER_SCAN_MAX 254

/* ================================================================ */
/*  录制参数                                                         */
/* ================================================================ */
#define TEST_SAMPLE_RATE       (16000)
#define TEST_RECORD_SECONDS    (5)
#define TEST_CHAN_NUM          (4)
#define TEST_I2S_BUF_FRAMES    (512)   /* 每次读取的帧数 */

/* ================================================================ */
/*  测试引脚定义 (来自 Netlist_Schematic1_1_2026-08-02.enet)          */
/* ================================================================ */
#define TEST_BUZZER_IO         (4)      /* BUZ, NPN(S8050) 驱动, 低电平响 */
/* UART1: GPIO12/13; UART2: GPIO47/48 (ESP32-S3 上 GPIO22~25 无效, 已弃用) */
#define TEST_UART1_TX          (12)
#define TEST_UART1_RX          (13)
#define TEST_UART2_TX          (47)
#define TEST_UART2_RX          (48)
/* IO1 排针 (47/48 已用作 UART2, 从 IO1 移除) */
#define TEST_IO1_PINS          {46, 1, 2}
#define TEST_IO1_PIN_COUNT     (3)
#define TEST_IO2_PINS          {42, 41, 40, 39, 38}
#define TEST_IO2_PIN_COUNT     (5)

/* 两个 I2C 引出排针 */
#define TEST_I2C1_SDA          (11)
#define TEST_I2C1_SCL          (10)
#define TEST_I2C2_SDA          (14)
#define TEST_I2C2_SCL          (7)

/* 两个 ADC: adc_3v3 与 adc5v */
#define TEST_ADC_3V3_IO        (6)   /* ADC1_CH5 */
#define TEST_ADC_5V_IO         (5)   /* ADC1_CH4 */

/* 麦克风阵列 (ES7210, I2S TDM 4ch) + RGB 灯环 */
#define TEST_WS2812_IO         (1)
#define TEST_LED_COUNT         (16)

/* ================================================================ */
/*  接口                                                             */
/* ================================================================ */
#define BOARD_TEST_TASK_STACK  (16384)   /* 自检任务栈: 含录音缓冲等大局部量 */
#define BOARD_TEST_TASK_PRIO   (5)

/* 在独立任务中启动板级自检 (避免占用 app_main 的默认小栈) */
void board_test_start(void);

#endif /* BOARD_TEST_H */