#pragma once

/* 灯效参数集中在本文件。改完重新编译烧录，不影响已经调好的主方向算法。
 * 亮度是麦克风收到的声强指示，不是距离：大声远源和小声近源可能一样亮。 */
#define LED_EFFECT_MAX_SOURCES       2       /* 四麦小阵列保守地同时显示最多两个方向 */
#define LED_EFFECT_FRAME_MS          20U     /* 50 Hz 刷新；新目标最多等待一个刷新周期 */
#define LED_EFFECT_BRIGHTNESS_MIN    8U      /* PWM 0~255：最弱可靠声源的峰值亮度 */
#define LED_EFFECT_BRIGHTNESS_MAX    80U     /* PWM 上限，留出供电余量，避免全白过亮 */
#define LED_EFFECT_ATTACK_MS         35.0f   /* 强度上升平滑时间；越小越跟手 */
#define LED_EFFECT_RELEASE_MS        180.0f  /* 声音变弱/消失时淡出时间 */
#define LED_EFFECT_FOLLOW_MS         32.0f   /* 连续角度的环形平滑时间常数，不逐灯停留 */
#define LED_EFFECT_TRAIL_MS          160U    /* 移动停止后相邻尾灯淡出时长 */
#define LED_EFFECT_TRAIL_RATIO       .10f    /* 尾灯最高为主灯10%，亮主灯始终指向声源 */
#define LED_EFFECT_JUMP_DEG          67.5f   /* 超过此角的突变另行确认，不模拟绕圈扫过 */
#define LED_EFFECT_LARGE_CONFIRM_MS  180U    /* 孤立大跳变持续确认时间 */
#define LED_EFFECT_ANGLE_DEAD_ZONE   .8f     /* 连续角度的小噪声死区，单位度 */
#define LED_EFFECT_STEP_CONFIRM_MS   120U    /* 静止时孤立阶跃确认，滤掉100ms来回跳 */
#define LED_EFFECT_PENDING_TOLERANCE_DEG 3.0f/* 孤立阶跃候选角容差 */
#define LED_EFFECT_MOTION_MIN_STEP   .25f    /* 统计连贯运动的最小输入步长，度 */
#define LED_EFFECT_MOTION_MAX_STEP   12.0f   /* 单次过大阶跃不算连贯运动，度 */
#define LED_EFFECT_MOTION_STEPS      3U     /* 连续同向的小步数，约40~60ms后直接跟随 */
#define LED_EFFECT_MOTION_HOLD_MS    160U    /* 输入偶有重复值，短时间继续保持运动状态 */
#define LED_EFFECT_HEAD_HYST_DEG     1.5f    /* 主灯跨扇区的轻微回差，防静止边界抖动 */
#define LED_EFFECT_STALE_MS          200U    /* 音频任务停止供数后自动熄灯，防止粘亮 */
#define LED_EFFECT_QUIET_DBFS        (-60.0f)/* 噪声扣除后的带内 RMS：强度映射下限 */
#define LED_EFFECT_LOUD_DBFS         (-25.0f)/* 强度映射上限；越低越容易达到最亮 */

/* 单方向显示模式：0=常亮，1=柔和呼吸（默认），2=闪烁。
 * 呼吸最低仍保持 70% 峰值，不用全灭闪烁干扰方向识别。 */
#define LED_EFFECT_MODE_STEADY        0
#define LED_EFFECT_MODE_BREATHE       1
#define LED_EFFECT_MODE_BLINK         2
#define LED_EFFECT_MODE              LED_EFFECT_MODE_BREATHE
#define LED_EFFECT_BREATH_PERIOD_MS  1800U
#define LED_EFFECT_BREATH_MIN        0.70f
#define LED_EFFECT_BLINK_PERIOD_MS   400U    /* 仅 MODE_BLINK 使用，50% 占空比 */

/* 0=随强度由蓝→青→绿→暖黄；1=使用下方固定 RGB。颜色分量范围 0~255。 */
#define LED_EFFECT_FIXED_COLOR       0
#define LED_EFFECT_FIXED_RED         0U
#define LED_EFFECT_FIXED_GREEN       150U
#define LED_EFFECT_FIXED_BLUE        255U

/* S3的RMT支持DMA：512个符号可容纳整条16灯帧，避免Wi-Fi/I2S忙时CPU
 * 来不及补充RMT小缓冲而产生与场景无关的乱码/散点闪光。失败时驱动报错。 */
#define LED_STRIP_USE_DMA             1
#define LED_STRIP_DMA_SYMBOLS         512U

#if LED_EFFECT_MODE < 0 || LED_EFFECT_MODE > 2
#error "LED_EFFECT_MODE must be STEADY, BREATHE or BLINK"
#endif
#if LED_EFFECT_BRIGHTNESS_MAX > 255 || LED_EFFECT_BRIGHTNESS_MIN > LED_EFFECT_BRIGHTNESS_MAX
#error "Invalid LED brightness range"
#endif
#if LED_EFFECT_MAX_SOURCES != 2
#error "This four-microphone visual tracker supports exactly two source slots"
#endif
