# DOA Single-Frame Outlier Rejection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent one high-confidence large-angle SRP result from changing the held DOA while preserving a two-frame response to a real sustained direction change.

**Architecture:** Keep filtering inside `doa_tracker`, before the angle reaches the web or LED. Track whether a pending jump is in the high-confidence two-frame path, clear that pending path on any invalid frame, and commit the circular mean directly after the second consistent candidate.

**Tech Stack:** C11, host-compiled tracker tests driven by pytest, ESP-IDF 5.4.2.

## Global Constraints

- Do not change SRP-PHAT scoring, bird classification, web display, source thresholds, or LED mapping.
- Candidates in one high-confidence jump must be within the existing 20-degree pending spread.
- Existing low-confidence pending and expiry behavior must remain unchanged.
- Work directly on the current `master` branch as authorized by the user.

---

### Task 1: Confirm High-Confidence Large Jumps

**Files:**
- Modify: `tests/test_doa_tracker_main.c`
- Modify: `main/doa_tracker.h`
- Modify: `main/doa_tracker.c`

**Interfaces:**
- Consumes: `doa_tracker_update(doa_tracker_t *, float, float, bool, float *)`
- Produces: unchanged public function signatures; `doa_tracker_t` gains private pending-path state.

- [ ] **Step 1: Write the failing regression test**

Replace the final immediate-jump expectation with behavior assertions equivalent to:

```c
const float stable = held;
if (doa_tracker_update(&tracker, 180.0f, 0.36f, true, &held)) return 13;
if (fabsf(held - stable) > 1.0f) return 14;
if (doa_tracker_update(&tracker, 0.0f, 0.0f, false, &held)) return 15;
if (fabsf(held - stable) > 1.0f) return 16;
if (doa_tracker_update(&tracker, 178.0f, 0.36f, true, &held)) return 17;
if (!doa_tracker_update(&tracker, 180.0f, 0.36f, true, &held)) return 18;
if (fabsf(held - 179.0f) > 5.0f) return 19;
```

- [ ] **Step 2: Run the tracker test and verify RED**

Run: `python -m pytest -q tests/test_doa_tracker.py`

Expected: FAIL because the first `180.0f, 0.36f` candidate is currently accepted immediately.

- [ ] **Step 3: Implement the minimal tracker change**

Add `pending_high_confidence_jump` to `doa_tracker_t`. Route movement above 45 degrees with confidence at least 0.35 through `accumulate_pending` with a required count of two. Reset this pending class on an invalid frame or a candidate from another pending class. On the second consistent high-confidence candidate, directly assign the averaged pending vector; leave the existing low-confidence smoothing path unchanged.

- [ ] **Step 4: Run focused and complete host tests**

Run: `python -m pytest -q tests/test_doa_tracker.py`

Expected: `1 passed`.

Run: `python -m pytest -q tests`

Expected: all tests pass.

- [ ] **Step 5: Build and commit**

Run after exporting ESP-IDF 5.4.2: `ninja -C build -j 2 all`

Expected: exit code 0 and `build/i2s_es7210_sound.bin` generated.

Commit:

```text
fix: reject single-frame doa jumps
```
