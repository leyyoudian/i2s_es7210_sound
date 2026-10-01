# DOA and bird-sound v2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the old classifier completely and improve four-microphone direction stability without hardware changes.

**Architecture:** Use the deploy-v2 Edge Impulse package as the sole model. Replace two-axis TDOA composition with a compact six-pair SRP-PHAT grid search that returns angle and confidence, then gate and smooth only valid results.

**Tech Stack:** ESP-IDF 5.4.2, C/C++, Edge Impulse TFLite Micro, host-side pytest plus TinyCC.

## Global Constraints

- Keep ES7210 capture at 16000 Hz with four TDM channels.
- Physical microphone positions are MIC1=270°, MIC2=180°, MIC3=0°, MIC4=90°, radius 25 mm.
- The build must not depend on `D:\Downlowads` after model import.
- Remove the old model and all HF confidence boosting from the active build.
- Invalid localization frames must not move the last reliable direction.

---

### Task 1: Import and validate deploy-v2 model

**Files:**
- Replace generated package: `bird-sound-cpp-mcu-v2/`
- Modify: `components/edge_impulse/CMakeLists.txt`
- Modify: `main/ei_inference.cpp`

**Interfaces:**
- Produces: `ei_run_inference(const int16_t *, int, ei_inference_result_t *)` backed only by deploy v2.

- [ ] Copy the complete downloaded deploy-v2 package into the project model directory and compare metadata hashes.
- [ ] Point `EI_BASE` and `EI_MODEL_FILE` to `bird-sound-cpp-mcu-v2/tflite-model/tflite_learn_966786_10.tflite`.
- [ ] Add compile-time assertions for 16000 Hz and two labels, and parse exact `bird` and `noise` label names.
- [ ] Reconfigure and build; verify boot metadata will report project `bird-sound`, deploy version 2, 15488 samples.

### Task 2: Six-pair SRP-PHAT core

**Files:**
- Modify: `main/gcc_phat.h`
- Modify: `main/gcc_phat.c`
- Create: `tests/test_srp_phat_main.c`
- Create: `tests/test_srp_phat.py`

**Interfaces:**
- Produces: `bool gcc_phat_estimate_4ch(const int16_t *mic[4], int n, int fs, gcc_phat_result_t *out)`.
- `gcc_phat_result_t` contains `angle_deg`, `score`, `second_score`, `confidence`, and `valid`.

- [ ] Write a host test generating delayed broadband signals for cardinal and diagonal angles; assert circular error is within one search bin.
- [ ] Run the test and verify it fails because the four-channel API does not exist.
- [ ] Compute one FFT per microphone, all six PHAT cross-correlations, and an SRP score over a 5° circular grid using actual microphone coordinates.
- [ ] Exclude the best peak neighborhood when finding the second peak and derive a normalized peak advantage.
- [ ] Reject low-energy, non-finite, weak-advantage results; interpolate only when the local score curvature is valid.
- [ ] Run the host test and tune only thresholds justified by the synthetic cases.

### Task 3: Persistent filtering and direction tracker

**Files:**
- Create: `main/doa_tracker.h`
- Create: `main/doa_tracker.c`
- Create: `tests/test_doa_tracker_main.c`
- Create: `tests/test_doa_tracker.py`
- Modify: `main/CMakeLists.txt`

**Interfaces:**
- Produces: `doa_tracker_update(doa_tracker_t *, float angle, float confidence, bool candidate_valid, float *held_angle)`.

- [ ] Write tests for 359°/1° wrapping, isolated 180° jumps, invalid-frame hold, and recovery after consecutive valid frames.
- [ ] Run tests and verify failure before implementation.
- [ ] Implement a fixed-size circular vector history and confidence gate without dynamic allocation.
- [ ] Run tests and verify all tracker cases pass.
- [ ] Move filter history into per-channel persistent state so each 1024-sample call continues from the previous frame.

### Task 4: Integrate audio, classification gates, and diagnostics

**Files:**
- Modify: `main/i2s_es7210_record_example.c`
- Modify: `main/ei_inference.h`

**Interfaces:**
- Consumes: deploy-v2 inference, `gcc_phat_estimate_4ch`, and `doa_tracker_update`.

- [ ] Replace the two-pair estimator call with the four-channel result and tracker.
- [ ] Calculate four-channel RMS and peak/clipping diagnostics before choosing the inference channel.
- [ ] Accumulate window RMS and bypass inference below the silence threshold.
- [ ] Remove `calc_hf_ratio_i16`, all HF constants, boost variables, and permissive negative-margin thresholds.
- [ ] Use conservative two-class hysteresis: activation requires bird probability greater than noise and a high raw threshold; release uses a lower threshold over the next result.
- [ ] Emit one localization summary per second plus one classifier summary per inference, including explicit rejection reason.
- [ ] Keep LED and UI direction frozen/off when localization is invalid.

### Task 5: Full verification

**Files:**
- Update if required: `dependencies.lock`, `sdkconfig`

- [ ] Run all host tests with `python -m pytest -q tests`.
- [ ] Run a clean ESP-IDF reconfigure and full build with `CCACHE_DISABLE=1`.
- [ ] Inspect binary size against the app partition; if needed select the built-in large single-app partition without changing flash size.
- [ ] Search the build map and source tree to verify the old model identifier and HF boost path are absent from active code.
- [ ] Document the four-step on-device validation: channel tap mapping, cardinal directions, music playback, and real bird playback.

The workspace is not a Git repository, so commit steps are intentionally omitted.
