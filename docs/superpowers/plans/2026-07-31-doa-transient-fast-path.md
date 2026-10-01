# DOA Transient Fast Path Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make gated transient sounds localize on their first valid frame while keeping ordinary single-frame direction outliers rejected.

**Architecture:** Add a transient-specific update function to `doa_tracker`. It immediately seeds the shared held vector and starts a six-frame guard that rejects only transient or large-angle reflection candidates. The audio task selects this function only for gated transient localization frames.

**Tech Stack:** C11, ESP-IDF FreeRTOS audio task, host tracker tests via pytest.

## Global Constraints

- Do not change bird classification, SRP-PHAT scoring, source thresholds, LED mapping, or web output.
- Keep ordinary high-confidence large jumps at two consistent frames.
- Keep existing low-confidence pending behavior unchanged.
- Work directly on current `master` as authorized by the user.

---

### Task 1: Add And Integrate Transient Tracker Path

**Files:**
- Modify: `tests/test_doa_tracker_main.c`
- Modify: `main/doa_tracker.h`
- Modify: `main/doa_tracker.c`
- Modify: `main/i2s_es7210_record_example.c`

**Interfaces:**
- Consumes: existing `doa_tracker_update` and audio-local `transient_doa` / `localization_valid` booleans.
- Produces: `doa_tracker_update_transient(doa_tracker_t *, float, float, bool, float *)`.

- [ ] **Step 1: Write the failing transient regression test**

After ordinary tracker tests initialize a stable angle, add assertions equivalent to:

```c
doa_tracker_init(&tracker, 0.15f);
if (doa_tracker_update(&tracker, 0.0f, 0.8f, true, &held)) return 31;
if (!doa_tracker_update(&tracker, 1.0f, 0.8f, true, &held)) return 32;
if (!doa_tracker_update_transient(&tracker, 270.0f, 0.36f, true, &held)) return 33;
if (fabsf(held - 270.0f) > 0.1f) return 34;
for (int i = 0; i < 6; ++i) {
    if (doa_tracker_update(&tracker, 90.0f, 0.8f, true, &held)) return 35;
    if (fabsf(held - 270.0f) > 0.1f) return 36;
}
if (doa_tracker_update(&tracker, 90.0f, 0.8f, true, &held)) return 37;
if (!doa_tracker_update(&tracker, 90.0f, 0.8f, true, &held)) return 38;
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `python -m pytest -q tests/test_doa_tracker.py`

Expected: FAIL to compile because `doa_tracker_update_transient` does not yet exist.

- [ ] **Step 3: Implement the tracker fast path**

Add `transient_guard_frames` to `doa_tracker_t` and define a six-frame guard. Implement `doa_tracker_update_transient` with the existing finite/confidence validation, direct circular-vector seeding, pending-state reset, and guard initialization. Decrement the guard on every ordinary update; while it is active, reject transient updates and ordinary candidates more than 45 degrees from the held direction, while allowing smaller ordinary movements through the existing smoothing path.

- [ ] **Step 4: Route gated transient frames through the new API**

In `audio_task`, call `doa_tracker_update_transient` when `transient_doa && localization_valid`; otherwise call the existing `doa_tracker_update`. Do not alter the transient detection thresholds.

- [ ] **Step 5: Run focused and complete tests**

Run: `python -m pytest -q tests/test_doa_tracker.py`

Expected: `1 passed`.

Run: `python -m pytest -q tests`

Expected: all tests pass.

- [ ] **Step 6: Build and commit**

Run after exporting ESP-IDF 5.4.2: `ninja -C build -j 2 all`

Expected: exit code 0 and `build/i2s_es7210_sound.bin` generated.

Commit message:

```text
fix: restore transient doa fast path
```
