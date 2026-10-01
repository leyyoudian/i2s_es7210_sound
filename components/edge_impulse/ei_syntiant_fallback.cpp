#include <stdlib.h>
#include <string.h>

#include "model-parameters/model_metadata.h"
#include "edge-impulse-sdk/dsp/numpy_types.h"
#include "edge-impulse-sdk/dsp/returntypes.hpp"
#include "esp_log.h"

static const char *TAG = "edge_impulse";

/*
 * Compatibility shim:
 * The exported v1 model expects extract_audio_syntiant_features(), but this
 * implementation is not shipped in the SDK bundle used by this project.
 *
 * To keep runtime stable, we output a fixed-size zero feature vector that
 * matches the model input shape and avoid experimental approximations here.
 */
int extract_audio_syntiant_features(ei::signal_t *signal, ei::matrix_t *output_matrix, void *config_ptr, const float sampling_frequency)
{
    (void)signal;
    (void)config_ptr;
    (void)sampling_frequency;

    if (!output_matrix || !output_matrix->buffer) {
        return ei::EIDSP_PARAMETER_INVALID;
    }

    const size_t capacity = output_matrix->rows * output_matrix->cols;
    if (capacity == 0) {
        return ei::EIDSP_MATRIX_SIZE_MISMATCH;
    }

    memset(output_matrix->buffer, 0, capacity * sizeof(float));
    output_matrix->rows = 1;
    output_matrix->cols = capacity;

    static bool printed = false;
    if (!printed) {
        ESP_LOGW(TAG,
                 "Using zero fallback for extract_audio_syntiant_features() (size=%u)",
                 (unsigned)capacity);
        printed = true;
    }

    return ei::EIDSP_OK;
}
