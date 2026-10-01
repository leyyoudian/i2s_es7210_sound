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
 * FFT 尺寸只影响频率分辨率, 1024 已覆盖 0.3-2kHz PHAT 频带.
 * 注意: 2048 点会使 dram0 静态区溢出 ~48KB, 无法链接. */
#define GCC_FFT_SIZE 1024

typedef struct {
    float angle_deg;
    float score;
    float second_score;
    float confidence;
    /* 对侧 (angle+180°) bin 与主峰的分数比: 十字阵列中心对称, 对径麦 80mm
     * 间距在 >2.14kHz 空间混叠, 镜像峰可能接近主峰. 比值接近 1 说明本帧
     * 方向含糊 (真实方向与对侧镜像难区分), 上层可据此拒绝瞬态快照. */
    float mirror_ratio;
    uint8_t active_mic_mask;
    uint8_t active_pair_count;
    bool valid;
} gcc_phat_result_t;

void gcc_phat_init(int frame_len);
/* 调整 SRP 分数轮廓 EMA 系数: 小值 = 长积分 (持续声源抗反射), 大值 = 快响应 */
void gcc_phat_set_score_ema_alpha(float alpha);

bool gcc_phat_estimate_4ch(const int16_t *mic[4], int n_samples,
                           int sample_rate, gcc_phat_result_t *out);

#ifdef __cplusplus
}
#endif
#endif
