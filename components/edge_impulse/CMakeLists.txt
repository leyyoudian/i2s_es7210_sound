cmake_minimum_required(VERSION 3.16)

# ------------------------------------------------------------------
# ESP-IDF component: edge_impulse
# Compiles Edge Impulse SDK + bird-sound model from local sources
# ------------------------------------------------------------------

set(EI_BASE "${CMAKE_CURRENT_LIST_DIR}/../../bird-sound-cpp-mcu-v2")
set(EI_SDK  "${EI_BASE}/edge-impulse-sdk")
set(EI_MODEL_FILE "${EI_BASE}/tflite-model/tflite_learn_992278_6.tflite")

# INCBIN resolves file paths at assembler time. Use absolute path to avoid
# failures when compiler temp files are emitted in a different working folder.
file(TO_CMAKE_PATH "${EI_MODEL_FILE}" EI_MODEL_FILE_CMAKE)

# ---- Collect source files ----

# DSP processing (MFCC, Syntiant, KissFFT)
file(GLOB_RECURSE EI_DSP_SRC   "${EI_SDK}/dsp/*.cpp")

# Classifier runner
file(GLOB EI_CLASSIFIER_SRC    "${EI_SDK}/classifier/*.cpp")

# TFLite Micro runtime
file(GLOB_RECURSE EI_TF_SRC
    "${EI_SDK}/tensorflow/*.cc"
    "${EI_SDK}/tensorflow/*.cpp"
    "${EI_SDK}/tensorflow/*.c")

# CMSIS-DSP math functions (needed for MFCC / FFT)
file(GLOB_RECURSE EI_CMSIS_DSP "${EI_SDK}/CMSIS/DSP/Source/*.c")

# CMSIS-NN kernels
file(GLOB_RECURSE EI_CMSIS_NN  "${EI_SDK}/CMSIS/NN/Source/*.c")

# ESP32 platform porting layer
file(GLOB EI_PORTING_SRC       "${EI_SDK}/porting/espressif/*.cpp")

# ESP-NN optimized kernels for Xtensa (ESP32-S3)
file(GLOB_RECURSE EI_ESPNN_SRC
    "${EI_SDK}/porting/espressif/ESP-NN/src/*.c"
    "${EI_SDK}/porting/espressif/ESP-NN/src/*.S")

# Edge Impulse bundled esp-dsp sources used by ei_esp_dsp backend
file(GLOB_RECURSE EI_ESPDSP_SRC
    "${EI_SDK}/porting/espressif/esp-dsp/modules/*.c"
    "${EI_SDK}/porting/espressif/esp-dsp/modules/*.cpp"
    "${EI_SDK}/porting/espressif/esp-dsp/modules/*.S")

# Model TFLite weights
file(GLOB MODEL_SRC            "${EI_BASE}/tflite-model/*.cpp")

# Local compatibility shims
set(EI_LOCAL_SRC
    "${CMAKE_CURRENT_LIST_DIR}/ei_syntiant_fallback.cpp")

# ---- Register component ----

idf_component_register(
    SRCS
        ${EI_DSP_SRC}
        ${EI_CLASSIFIER_SRC}
        ${EI_TF_SRC}
        ${EI_CMSIS_DSP}
        ${EI_CMSIS_NN}
        ${EI_PORTING_SRC}
        ${EI_ESPNN_SRC}
        ${EI_ESPDSP_SRC}
        ${MODEL_SRC}
        ${EI_LOCAL_SRC}
    INCLUDE_DIRS
        "${EI_BASE}"
        "${EI_SDK}"
        "${EI_SDK}/third_party/flatbuffers/include"
        "${EI_SDK}/third_party/gemmlowp"
        "${EI_SDK}/third_party/ruy"
        "${EI_SDK}/CMSIS/Core/Include"
        "${EI_SDK}/CMSIS/DSP/Include"
        "${EI_SDK}/CMSIS/DSP/PrivateInclude"
        "${EI_SDK}/CMSIS/NN/Include"
        "${EI_SDK}/porting/espressif/ESP-NN/include"
        "${EI_SDK}/porting/espressif/ESP-NN/src/common"
        "${EI_SDK}/porting/espressif/esp-dsp/modules/common/include"
        "${EI_SDK}/porting/espressif/esp-dsp/modules/fft/include"
    REQUIRES esp_timer
)

# ---- Compile definitions ----
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    EI_PORTING_ESPRESSIF=1
    EI_CLASSIFIER_TFLITE_ENABLE_ESP_NN=0
    EIDSP_USE_ESP_DSP=0
    __PROGRAM_START=0
    TF_LITE_DISABLE_X86_NEON=1
    EI_TFLITE_MODEL_PATH="${EI_MODEL_FILE_CMAKE}"
    portTICK_RATE_MS=portTICK_PERIOD_MS
)

# Suppress third-party code warnings
target_compile_options(${COMPONENT_LIB} PRIVATE
    -w
)
