#ifndef EI_INFERENCE_H
#define EI_INFERENCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 推理结果结构体 */
typedef struct {
    float bird_confidence;   /* 鸟声置信度 [0.0, 1.0] */
    float noise_confidence;  /* 噪声置信度 [0.0, 1.0] */
    int   predicted_class;   /* 0 = bird, 1 = noise */
    float dsp_time_ms;       /* DSP 耗时 (ms) */
    float classify_time_ms;  /* 神经网络推理耗时 (ms) */
} ei_inference_result_t;

/*
 * 初始化推理引擎, 打印模型信息
 * 返回 0 成功
 */
int ei_inference_init(void);

/*
 * 对一段原始音频运行分类推理
 *   samples:   16-bit PCM 单声道, 16kHz
 *   n_samples: 采样数, 应等于 15488
 *   result:    输出结果
 * 返回 0 成功, -1 失败
 */
int ei_run_inference(const int16_t *samples, int n_samples,
                     ei_inference_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* EI_INFERENCE_H */
