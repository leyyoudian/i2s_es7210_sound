/*
 * 例程选择开关 (Examples Selector)
 *
 * 本工程自带一组官方例程风格的外设演示, 每次烧录运行其中一个.
 * 使用方法: 去掉下面某一行前面的 "//" 即启用该例程;
 * 全部保持注释则运行总合演示 (声源定位 + 鸟声识别).
 *
 *  ┌────────────────────┬─────────────────────────────────────────────────────┐
 *  │ 宏                  │ 例程              │ 演示内容                          │
 *  ├────────────────────┼───────────────────┼──────────────────────────────────┤
 *  │ RUN_EXAMPLE_I2C_SCAN│ I2C 总线扫描     │ 扫描板上两路 I2C 排针 (SDA/SCL)   │
 *  │ RUN_EXAMPLE_ADC     │ ADC 电压读取     │ 读取两路 ADC 排针电压 (GPIO5/6)    │
 *  │ RUN_EXAMPLE_UART    │ UART 互通回环    │ 两路 UART 交叉互发 (GPIO12/13,     │
 *  │                     │                  │ 47/48)                           │
 *  │ RUN_EXAMPLE_RGB     │ RGB 灯环         │ WS2812 彩虹流水灯 (GPIO1)         │
 *  │ RUN_EXAMPLE_ES7210  │ 麦克风阵列录音   │ ES7210 4ch 采集与电平分析          │
 *  │ RUN_EXAMPLE_USB     │ USB 串口         │ USB Serial/JTAG 回显 (内置)       │
 *  │ RUN_EXAMPLE_BLE     │ 蓝牙 BLE         │ NimBLE 广播 + GATT 回显 (内置)    │
 *  │ RUN_EXAMPLE_BOARD_TEST│ 板级全面自检   │ 依次测试全部板载外设               │
 *  └────────────────────┴───────────────────┴──────────────────────────────────┘
 *
 * ═══════════════════════════ 板载资源 · 引脚速查 ═══════════════════════════
 *  │ 功能          │ 引脚/接口                                     │ 说明        │
 *  │ 4×MEMS 麦克风 │ I2S: MCK=16 BCK=18 WS=17 DI=15               │ ES7210 TDM  │
 *  │ ES7210 控制   │ I2C0: SDA=8 SCL=9 (地址 0x40)                │ 48kHz 16bit │
 *  │ 16×WS2812 灯环│ GPIO1                                        │ 方向指示    │
 *  │ 无源蜂鸣器    │ GPIO4 (NPN 驱动, 低电平响)                    │ 4kHz PWM    │
 *  │ UART1         │ TX=12  RX=13                                 │ 排针引出    │
 *  │ UART2         │ TX=47  RX=48                                 │ 排针引出    │
 *  │ I2C1 排针     │ SDA=11 SCL=10                                │ 外接传感器  │
 *  │ I2C2 排针     │ SDA=14 SCL=7                                 │ 外接传感器  │
 *  │ ADC1_CH5      │ GPIO6 (adc_3v3, 0~3.3V)                      │ 排针引出    │
 *  │ ADC1_CH4      │ GPIO5 (adc5v,  0~5V 分压)                    │ 排针引出    │
 *  │ IO1 排针      │ GPIO46/1/2                                   │ 数字 IO     │
 *  │ IO2 排针      │ GPIO42/41/40/39/38                           │ 数字 IO     │
 *  │ USB-C         │ 内置 USB Serial/JTAG (GPIO19/20 物理)        │ 下载/串口   │
 *  │ 蓝牙          │ 内置 BLE 5.0 (与 Wi-Fi 共用天线)              │ 无需外接    │
 *  ════════════════╧══════════════════════════════════════════════╧═════════════
 */

#ifndef EXAMPLES_H
#define EXAMPLES_H

//#define RUN_EXAMPLE_I2C_SCAN
//#define RUN_EXAMPLE_ADC
//#define RUN_EXAMPLE_UART
//#define RUN_EXAMPLE_RGB
//#define RUN_EXAMPLE_ES7210
//#define RUN_EXAMPLE_USB
//#define RUN_EXAMPLE_BLE
//#define RUN_EXAMPLE_BOARD_TEST

void example_i2c_scan_start(void);
void example_adc_start(void);
void example_uart_start(void);
void example_rgb_start(void);
void example_es7210_start(void);
void example_usb_start(void);
void example_ble_start(void);

#endif /* EXAMPLES_H */