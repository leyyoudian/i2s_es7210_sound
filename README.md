# 声源定位 · 鸟声识别开发板

**ESP32-S3 + ES7210 四麦克风阵列 · 端侧智能声学开发板**

一块同时具备 **实时声源定位** 与 **端侧声音识别** 的开发板。配套固件采用官方例程式组织，外设例程一应俱全，开箱即可玩，二次开发极友好。

> 🚀 **刚拿到板子？** 从 [docs/使用指南.md](docs/使用指南.md) 开始：开箱 → 上电 → 连 Wi-Fi → 手机看面板，5 分钟跑起来。

---

## ✨ 核心卖点

### 🎯 声源定位（DOA）
- **4 颗 SPH1642HT5H MEMS 麦克风**，十字阵列，40mm 孔径
- **48kHz 全速率同步采集**（I2S TDM 4 通道），时差分辨率达微秒级
- **GCC-PHAT + SRP-PHAT 双算法**：互相关 + 可控响应功率全向搜索
- **2° 全向搜索栅格**，0~360° 连续方位角输出
- **智能跟踪器**：方向平滑 + 反射假峰抑制 + 瞬态事件快速路径（拍手即响应）
- **16 颗 WS2812 灯环实时指向**声源方位，一眼可见

### 🐦 端侧声音识别
- **Edge Impulse 训练 + TFLite Micro INT8 量化模型**，推理在板端完成，**不依赖网络**
- 48kHz → 16kHz **12 阶 FIR 抗混叠抽取**，喂给模型的每一帧都是干净数据
- 自动选通最强信号通道 + 自适应增益（AGC），远距离弱鸟叫也能召回
- 分类平滑 + 静音门控 + 决策状态机，**抗误报**，不会风吹草动就乱报

### 📱 手机实时面板
- 板载 Wi-Fi，手机浏览器直接打开设备 IP
- 罗盘式方向显示 + 鸟声/噪声置信度实时刷新（100ms 节奏）
- 无需 App、无需云端，局域网即连即用

### 🔧 官方例程式代码组织
- `main/examples/` 独立例程：**I2C 扫描 / ADC 读取 / UART 互通 / RGB 灯环 / ES7210 录音 / USB 串口 / 蓝牙 BLE**
- 一键切换：编辑 `examples.h` 注释开关，重新烧录即运行
- 完整板级自检（`RUN_EXAMPLE_BOARD_TEST`）：Wi-Fi、I2C×2、ADC×2、RGB、蜂鸣器、UART×2、IO 排针、麦克风阵列逐项 **PASS/FAIL**，麦克风录音自动上传 PC **在线回放**
- 生产测试、教学演示、二次开发，全都开箱即用

---

## 📐 核心指标

| 项目 | 指标 |
| ---- | ---- |
| 主控 | ESP32-S3（双核 240MHz, Wi-Fi + USB OTG） |
| 音频前端 | ES7210 4 通道音频 ADC（I2C 配置，TDM 输出） |
| 麦克风 | 4 × SPH1642HT5H MEMS，十字阵列 40mm 孔径 |
| 采样率 | 48kHz / 16bit / 4 通道同步 |
| 方位估计 | GCC-PHAT 预滤波 + SRP-PHAT 2° 全向搜索 |
| 方向输出 | 0~360° 连续角度 + 16 灯环方位指示 |
| 识别模型 | TFLite Micro INT8，16kHz 输入（板端抗混叠抽取） |
| 指示灯 | 16 × WS2812 RGB（GPIO1） |
| 面板 | Wi-Fi + HTTP Server，手机浏览器即开即用 |
| 接口 | 2×I2C、2×UART、5+3 路 IO 排针、2×ADC、USB-C |
| 板级自检 | 全外设逐项 PASS/FAIL + 录音上传回放 |

## 📦 例程列表

| 例程 | 文件 | 说明 |
| ---- | ---- | ---- |
| I2C 总线扫描 | `examples/example_i2c_scan.c` | 扫描两路 I2C 排针，打印应答设备地址 |
| ADC 电压读取 | `examples/example_adc.c` | 双路 ADC 电压实时打印（含 5V 分压还原） |
| UART 双路互通 | `examples/example_uart.c` | 两路 UART 交叉互发 hello/world |
| RGB 灯环 | `examples/example_rgb.c` | 彩虹流水 / 白色呼吸 / 红色追逐 |
| 麦克风阵列录音 | `examples/example_es7210.c` | 4ch TDM 采集，逐通道电平分析 |
| USB 串口 | `examples/example_usb.c` | USB Serial/JTAG 回显，一根线直连 PC |
| 蓝牙 BLE | `examples/example_ble.c` | NimBLE 广播 + GATT 回显服务，手机直连 |
| **总合演示**（默认固件） | `main/main.c` | 鸟声识别 + 声源定位 + 灯环指向 + Web 面板 |
| 板级全面自检 | `RUN_EXAMPLE_BOARD_TEST` | 全外设逐项检测 + 录音上传 |

**切换方式**：编辑 `main/examples/examples.h`，去掉对应 `RUN_EXAMPLE_xxx` 的注释，重新编译烧录。

## 📁 目录结构

```
main/                 # 主程序入口是 main/main.c (含 app_main)
├── main.c            总合演示：识别 + 定位（默认固件入口）
├── doa/              声源定位：GCC-PHAT/SRP、跟踪器、事件检测、门控、方向映射
├── bird_ai/          声音识别：Edge Impulse 推理封装 + 决策平滑
├── led/              WS2812 灯环驱动
├── wifi/             Wi-Fi 连接 + HTTP 服务器 + Web 罗盘面板
├── board_test/       板级生产自检（逐项 PASS/FAIL + 录音上传）
└── examples/         独立外设例程（i2c/adc/uart/rgb/es7210/usb/ble）
```

## 🚀 快速上手

1. USB-C 线连接开发板与电脑
2. 安装 [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/)（v5.4 及以上）
3. 编译烧录：

```bash
idf.py set-target esp32s3
idf.py -p PORT flash monitor
```

4. 烧录后按串口日志提示连接 Wi-Fi，手机浏览器打开设备 IP 即可看到实时面板
5. 想跑单个例程？编辑 `main/examples/examples.h` 切换后重新烧录

## 🔧 硬件引脚速查

| 外设 | 引脚 |
| ---- | ---- |
| ES7210 I2C | SDA=GPIO8, SCL=GPIO9 |
| ES7210 I2S | MCLK=16, BCLK=18, WS=17, DIN=15 |
| WS2812 灯环 | GPIO1（16 颗） |
| 蜂鸣器 | GPIO4（NPN 驱动，低电平响） |
| ADC 排针 | GPIO6（3V3 直连）、GPIO5（5V 经 22k/39k 分压） |
| I2C 排针 | I2C1: SDA=11/SCL=10，I2C2: SDA=14/SCL=7 |
| UART 排针 | UART1: TX=12/RX=13，UART2: TX=47/RX=48 |
| IO 排针 | IO1: 46/1/2，IO2: 42/41/40/39/38 |
| USB-C | 内置 USB Serial/JTAG（GPIO19=D-, GPIO20=D+） |
| 蓝牙 | 内置 BLE 5.0，与 Wi-Fi 共用天线，无需接线 |

## 🛠️ 二次开发建议

- **换识别对象**：在 [Edge Impulse](https://studio.edgeimpulse.com/) 用自己的数据训练新模型，替换 `bird-sound-cpp-mcu-v2/tflite-model/` 即可（狗叫、猫叫、口哨、玻璃破碎……都是现成思路）
- **调定位参数**：阵列几何在 `main/doa/gcc_phat.h`（`MIC_ARRAY_RADIUS_MM`），搜索栅格在 `gcc_phat.c`（`SRP_STEP_DEG`）
- **换指示方式**：`main/led/led_indicator.c`，16 灯的物理角度表在 `main/doa/direction_mapping.c`
- **面板改版**：`main/wifi/web_page.html`，嵌入固件随设备下发

---

## 板级自检模式（生产测试）

编辑 `main/examples/examples.h` 启用 `RUN_EXAMPLE_BOARD_TEST` 后烧录，固件将依次检测：

Wi-Fi 连接 → I2C×2 排针扫描 → ADC×2 读取 → RGB 灯环 → 蜂鸣器 → UART×2 + IO 排针 → 麦克风阵列录音上传。

录音经 HTTP 上传到 PC（`python tools/upload_server.py`），浏览器打开 `http://<PC_IP>:8080/` 即可**在线回放**每一通道的录音，快速定位坏麦。每项外设独立 PASS/FAIL，产线可直接用作出货检验。

## ⚠️ 注意事项

- 自检固件二进制约 1.1MB，工程使用自定义分区表（`partitions.csv`，app 分区 2MB）
- GCC 14 编译 Edge Impulse SDK 偶发内部编译器错误（ICE），重跑 `idf.py build` 即可
- 本板不含 DAC，ES7210 仅做录音（无回放）
