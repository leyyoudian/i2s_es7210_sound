#ifndef GCC_PHAT_H
#define GCC_PHAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MIC_ARRAY_RADIUS_MM 40.0f
#define SOUND_SPEED_MPS 343.0f
/* 48kHz 采集, 1024 点 = 21.3ms 分析帧. TDOA 分辨率由采样率决定 (比 16kHz x3);
 * FFT 尺寸只影响频率分辨率, 1024 已覆盖 0.3-6kHz PHAT 频带.
 * 注意: 2048 点会使 dram0 静态区溢出 ~48KB, 无法链接. */
#define GCC_FFT_SIZE 1024
#define GCC_MAX_DISPLAY_PEAKS 2 /* 仅供多方向灯效，主跟踪器仍保持原先单方向逻辑 */

typedef struct {
    float angle_deg;       /* SRP 原始坐标，进入灯效前需 direction_calibrate_doa */
    float score;           /* PHAT 空间峰，不等于声压或距离 */
    float prominence;      /* 峰相对两侧谷底的突出度，0~1 */
    float directional_rms; /* 延时对齐四通道后实际 RMS，归一化到 int16 满量程 */
} gcc_phat_peak_t;

typedef struct {
    float angle_deg;
    float score;
    float second_score;
    float confidence;
    /* Same integrated correlation, evaluated at zero pair delays. Common
     * mode interference can be loud but carries no azimuth evidence. */
    float zero_delay_score;
    /* 对侧 (angle+180°) bin 与主峰的分数比: 十字阵列中心对称, 对径麦 80mm
     * 间距在 >2.14kHz 空间混叠, 镜像峰可能接近主峰. 比值接近 1 说明本帧
     * 方向含糊 (真实方向与对侧镜像难区分), 上层可据此拒绝瞬态快照. */
    float mirror_ratio;
    /* Current frame only: transients must not inherit a previous source's EMA. */
    float instant_angle_deg;
    float instant_score;
    float instant_confidence;
    float instant_mirror_ratio;
    uint8_t active_mic_mask;
    uint8_t active_pair_count;
    bool valid;
    uint8_t display_peak_count;
    gcc_phat_peak_t display_peaks[GCC_MAX_DISPLAY_PEAKS];
} gcc_phat_result_t;

void gcc_phat_init(int frame_len);
void gcc_phat_reset_history(void);
void gcc_phat_set_noise_learning(bool enabled);
void gcc_phat_set_instant_enabled(bool enabled);
/* 调整 SRP 分数轮廓 EMA 系数: 小值 = 长积分 (持续声源抗反射), 大值 = 快响应 */
void gcc_phat_set_score_ema_alpha(float alpha);

bool gcc_phat_estimate_4ch(const int16_t *mic[4], int n_samples,
                           int sample_rate, gcc_phat_result_t *out);

#ifdef __cplusplus
}
#endif
#endif
