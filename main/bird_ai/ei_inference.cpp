/*
 * Edge Impulse 推理包装层
 * 接收 int16_t PCM 音频, 转换为 float 信号, 送入 EI 分类器
 * 模型: bird-sound (bird / noise 二分类, INT8 量化)
 */

#include "ei_inference.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "ei_infer";

static_assert(EI_CLASSIFIER_FREQUENCY == 16000,
              "bird-sound deploy v2 must use 16 kHz PCM");
static_assert(EI_CLASSIFIER_LABEL_COUNT >= 2,
              "bird-sound deploy v2 must contain bird species and noise labels");

/* 音频缓冲区指针, 供信号回调读取 */
static const int16_t *s_audio_ptr = nullptr;
static int s_audio_len = 0;

/*
 * 信号数据回调: EI 分类器调用此函数分批获取音频数据.
 * 将 int16 采样值归一化到 [-1.0, 1.0] 浮点范围.
 */
static int get_signal_data(size_t offset, size_t length, float *out)
{
    for (size_t i = 0; i < length; i++) {
        if (offset + i < (size_t)s_audio_len) {
            out[i] = (float)s_audio_ptr[offset + i] / 32768.0f;
        } else {
            out[i] = 0.0f;
        }
    }
    return 0;
}

extern "C" int ei_inference_init(void)
{
    ESP_LOGI(TAG, "模型: %s (v%d)", EI_CLASSIFIER_PROJECT_NAME,
             EI_CLASSIFIER_PROJECT_DEPLOY_VERSION);
    ESP_LOGI(TAG, "输入: %d samples @ %d Hz, 类别: %d",
             EI_CLASSIFIER_RAW_SAMPLE_COUNT,
             EI_CLASSIFIER_FREQUENCY,
             EI_CLASSIFIER_LABEL_COUNT);
    ESP_LOGI(TAG, "量化: INT8, 推理引擎: TFLite Micro, Arena: %d bytes",
             EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE);
    return 0;
}

extern "C" int ei_run_inference(const int16_t *samples, int n_samples,
                                ei_inference_result_t *result)
{
    if (n_samples < EI_CLASSIFIER_RAW_SAMPLE_COUNT) {
        ESP_LOGE(TAG, "缓冲区不足: %d < %d", n_samples,
                 EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return -1;
    }

    /* 设置信号回调的数据源 */
    s_audio_ptr = samples;
    s_audio_len = n_samples;

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &get_signal_data;

    ei_impulse_result_t ei_res = {};

    /* 运行完整推理流水线: DSP特征提取 -> NN推理 -> 后处理 */
    EI_IMPULSE_ERROR err = run_classifier(&signal, &ei_res, false);
    if (err != EI_IMPULSE_OK) {
        ESP_LOGE(TAG, "分类器错误: %d", (int)err);
        return -1;
    }

    float bird_val = 0.0f;
    float noise_val = -1.0f;
    bool found_bird_species = false;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        const char *label = ei_res.classification[i].label;
        float val = ei_res.classification[i].value;
        if (label && strcmp(label, "noise") == 0) {
            noise_val = val;
        } else if (label && (strcmp(label, "housesparrow") == 0 ||
                             strcmp(label, "roseringedparakeet") == 0 ||
                             strcmp(label, "bird") == 0)) {
            bird_val += val;
            found_bird_species = true;
        }
    }

    if (!found_bird_species || noise_val < 0.0f) {
        ESP_LOGE(TAG, "模型标签不匹配: 必须包含 bird 和 noise");
        return -1;
    }

    result->bird_confidence  = bird_val;
    result->noise_confidence = noise_val;
    result->predicted_class  = (result->bird_confidence > result->noise_confidence) ? 0 : 1;
    result->dsp_time_ms      = ei_res.timing.dsp;
    result->classify_time_ms = ei_res.timing.classification;

    ESP_LOGI(TAG, "bird=%.1f%% noise=%.1f%% (DSP %.0fms, NN %.0fms)",
             result->bird_confidence * 100.0f,
             result->noise_confidence * 100.0f,
             result->dsp_time_ms,
             result->classify_time_ms);

    return 0;
}
