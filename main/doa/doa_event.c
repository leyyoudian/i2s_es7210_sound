#include "doa_event.h"

#include <math.h>

#define STRONG_SNAP_FRAME_RMS_MIN          (0.0100f)              /* 强瞬态整帧归一化RMS最低值 */
#define STRONG_SNAP_TRANSIENT_MIN          (0.0300f)              /* 强瞬态2ms短块RMS最低值 */
#define STRONG_SNAP_PEAK_RATIO_MIN         (2.25f)                /* 强瞬态短块/整帧RMS比下限 */
#define STRONG_SNAP_SRP_SCORE_MIN          (0.0250f)              /* 强瞬态空间分数下限，强声仍需真实置信 */

#define WEAK_TAP_FRAME_RMS_MIN             (0.0015f)              /* 弱敲击整帧归一化RMS最低值 */
#define WEAK_TAP_TRANSIENT_MIN             (0.0045f)              /* 弱敲击2ms短块RMS最低值 */
#define WEAK_TAP_PEAK_RATIO_MIN            (1.60f)                /* 弱敲击短块/整帧RMS比下限 */
#define WEAK_TAP_SRP_SCORE_MIN             (0.0250f)              /* 弱敲击当前帧空间分数最低值 */
#define WEAK_TAP_SRP_CONF_MIN              (0.1600f)              /* 弱敲击当前帧置信最低值 */

/* 低声压补充路径：归一化RMS、空间分数、置信、镜像、共模差值同时满足。
 * 不建议为追求“有灯”单独降低某项，否则环境噪声容易出现随机方向。 */
#define WEAK_SPATIAL_RMS_MIN        .0003f /* int16满量程归一化带内RMS绝对下限 */
#define WEAK_SPATIAL_RMS_RATIO      1.3f   /* 带内RMS/背景RMS，不是功率比 */
#define WEAK_SPATIAL_SCORE_MIN      .010f  /* PHAT相关分数，非声压和概率 */
#define WEAK_SPATIAL_CONF_MIN       .16f   /* 空间峰相对其它独立峰的区分度 */
#define WEAK_SPATIAL_MIRROR_MAX     .60f   /* 对侧分数/主峰分数上限 */
#define WEAK_SPATIAL_COMMON_MARGIN  .0005f /* 方向峰须超过零延时共模的最小差值 */

bool doa_event_has_weak_spatial_evidence(bool srp_valid, float band_rms,
                                         float noise_rms, float score,
                                         float zero_delay_score, float confidence,
                                         float mirror_ratio)
{
    return srp_valid && isfinite(band_rms) && isfinite(noise_rms) &&
           isfinite(score) && isfinite(zero_delay_score) &&
           isfinite(confidence) && isfinite(mirror_ratio) && noise_rms > 0.0f &&
           band_rms >= fmaxf(WEAK_SPATIAL_RMS_MIN, noise_rms * WEAK_SPATIAL_RMS_RATIO) &&
           score >= WEAK_SPATIAL_SCORE_MIN && confidence >= WEAK_SPATIAL_CONF_MIN &&
           mirror_ratio < WEAK_SPATIAL_MIRROR_MAX &&
           score > zero_delay_score + WEAK_SPATIAL_COMMON_MARGIN;
}

bool doa_event_can_lock_single_frame(bool has_direction, float band_rms,
                                     float noise_rms)
{
    if (!isfinite(band_rms) || !isfinite(noise_rms) ||
        band_rms <= 0.0f || noise_rms <= 0.0f) return false;
    /* With no established direction, confidence cannot distinguish a single
     * random noise peak from a real isolated event. Weak continuous sources
     * can still acquire through the ordinary multi-frame tracker. */
    return band_rms >= noise_rms * (has_direction ? 1.8f : 3.0f);
}

bool doa_event_needs_instant_analysis(bool source_active, float frame_rms,
                                     float previous_frame_rms, float transient_rms,
                                     float previous_transient_rms, float noise_rms)
{
    if (!source_active) return true;
    if (!isfinite(frame_rms) || !isfinite(previous_frame_rms) ||
        !isfinite(transient_rms) || !isfinite(previous_transient_rms) ||
        !isfinite(noise_rms)) return false;
    /* Require an actual energy rise above the preceding frame AND noise.
     * This leaves the second FFT/profile pass off during steady music. */
    return (frame_rms >= fmaxf(previous_frame_rms * 2.5f, noise_rms * 4.0f)) ||
           (transient_rms >= fmaxf(previous_transient_rms * 3.0f, noise_rms * 8.0f));
}

bool doa_event_is_strong_snap(float frame_rms, float transient_rms,
                              float peak_ratio, float srp_score)
{
    return isfinite(frame_rms) && isfinite(transient_rms) &&
           isfinite(peak_ratio) && isfinite(srp_score) &&
           transient_rms >= STRONG_SNAP_TRANSIENT_MIN &&
           frame_rms >= STRONG_SNAP_FRAME_RMS_MIN &&
           peak_ratio >= STRONG_SNAP_PEAK_RATIO_MIN &&
           srp_score >= STRONG_SNAP_SRP_SCORE_MIN;
}

bool doa_event_use_fast_path(float frame_rms, float transient_rms,
                             float peak_ratio, float srp_score,
                             float srp_confidence)
{
    if (doa_event_is_strong_snap(frame_rms, transient_rms,
                                 peak_ratio, srp_score) &&
        isfinite(srp_confidence) && srp_confidence >= WEAK_TAP_SRP_CONF_MIN) {
        return true;
    }

    return isfinite(frame_rms) && isfinite(transient_rms) &&
           isfinite(peak_ratio) && isfinite(srp_score) &&
           isfinite(srp_confidence) &&
           frame_rms >= WEAK_TAP_FRAME_RMS_MIN &&
           transient_rms >= WEAK_TAP_TRANSIENT_MIN &&
           peak_ratio >= WEAK_TAP_PEAK_RATIO_MIN &&
           srp_score >= WEAK_TAP_SRP_SCORE_MIN &&
           srp_confidence >= WEAK_TAP_SRP_CONF_MIN;
}
