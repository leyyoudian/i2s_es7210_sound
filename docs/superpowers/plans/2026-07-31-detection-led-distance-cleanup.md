# Detection, LED, and Distance Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Tighten bird-state decisions, normalize web angles, guarantee single-pixel LED refreshes, and remove the misleading RMS distance feature.

**Architecture:** Preserve the existing EI inference, SRP-PHAT, DOA tracker, direction mapping, and task split. Change only the bird decision state machine, the dashboard/SSE contract, and the LED strip output boundary; remove the standalone distance estimator and all consumers.

**Tech Stack:** ESP-IDF C/C++, Edge Impulse v2 model, ESP-IDF `led_strip`, embedded HTML/JavaScript, pytest, MSVC host-side C tests, Node.js for dashboard JavaScript evaluation.

## Global Constraints

- Keep `bird-sound-cpp-mcu-v2` unchanged.
- Keep two-window bird entry confirmation.
- Use entry thresholds `0.68`, `0.58`, and raw margin `0.16`.
- Exit bird immediately when raw bird confidence is `0.35` or lower.
- Display angles in `[0, 360)` and keep the canvas pointer formula unchanged.
- Require three consecutive requests before changing LEDs.
- Send an all-off LED frame before every single-pixel frame.
- Remove distance estimation completely; do not replace or hide it.
- Do not change microphone geometry, source-presence thresholds, SRP-PHAT, DOA calibration, LED angle mapping, brightness, or color.

---

### Task 1: Bird Decision Thresholds and Fast Exit

**Files:**
- Modify: `tests/test_bird_decision_main.c`
- Modify: `main/bird_decision.c`

**Interfaces:**
- Consumes: `bird_decision_update(bird_decision_t *, float, float, float, float)`
- Produces: the same API with stricter entry and immediate low-confidence exit behavior

- [ ] **Step 1: Write the failing bird-decision cases**

Replace the obsolete `0.66/0.67` relaxed-entry case with literal midpoint-boundary cases, then add a state-exit case:

```c
bird_decision_init(&state);
if (bird_decision_update(&state, 0.67f, 0.33f, 0.60f, 0.40f)) return 12;
if (bird_decision_update(&state, 0.68f, 0.32f, 0.57f, 0.43f)) return 13;
if (bird_decision_update(&state, 0.68f, 0.32f, 0.58f, 0.42f)) return 14;
if (!bird_decision_update(&state, 0.70f, 0.30f, 0.60f, 0.40f)) return 15;

if (bird_decision_update(&state, 0.332f, 0.668f, 0.55f, 0.45f)) return 16;
```

The first two calls prove each threshold rejects independently, the third and fourth prove two qualifying windows are still required, and the final call reproduces the logged `33.2%` stale-bird failure.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `python -m pytest -q tests/test_bird_decision.py`

Expected: FAIL because the current `0.65/0.55/0.12` entry accepts an underspecified window and the current immediate-exit cutoff is only `0.20`.

- [ ] **Step 3: Implement the minimal state-machine change**

In `main/bird_decision.c`, set:

```c
#define BIRD_THRESHOLD 0.68f
#define BIRD_SMOOTH_THRESHOLD 0.58f
#define BIRD_MARGIN 0.16f
#define BIRD_IMMEDIATE_EXIT_THRESHOLD 0.35f
```

Use `raw_bird <= BIRD_IMMEDIATE_EXIT_THRESHOLD` in the existing immediate-exit branch. Leave `CONFIRM_WINDOWS`, strong-noise handling, and the ordinary noise hysteresis unchanged.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run: `python -m pytest -q tests/test_bird_decision.py`

Expected: `1 passed`.

- [ ] **Step 5: Commit the bird-decision change**

```powershell
git add -- main/bird_decision.c tests/test_bird_decision_main.c
git commit -m "tighten bird decision hysteresis"
```

### Task 2: Unsigned Web Angle and End-to-End Distance Removal

**Files:**
- Modify: `tests/test_web_compass.py`
- Modify: `main/web_page.html`
- Modify: `main/wifi_server.h`
- Modify: `main/wifi_server.c`
- Modify: `main/i2s_es7210_record_example.c`
- Modify: `main/CMakeLists.txt`
- Delete: `main/distance_estimator.c`
- Delete: `main/distance_estimator.h`
- Delete: `tests/test_distance_estimator.py`
- Delete: `tests/test_distance_estimator_main.c`

**Interfaces:**
- Consumes: calibrated `angle`, bird/noise confidence, and detected state
- Produces: `wifi_server_push_data(float angle, float bird_conf, float noise_conf, bool bird_detected)` and SSE JSON with no distance field

- [ ] **Step 1: Write the failing dashboard behavior test**

Extend `tests/test_web_compass.py` with a Node-backed evaluation of the real embedded script. Provide minimal `document`, canvas context, and `EventSource` fakes, evaluate the script, and assert:

```python
assert evaluate_display_angle(-90.0) == 270.0
assert evaluate_display_angle(360.0) == 0.0
assert evaluate_display_angle(315.0) == 315.0
```

The helper must execute the page's `displayAngle()` JavaScript function rather than reimplementing normalization in Python. Also assert from the rendered HTML contract that there is no element with ID `distVal` and no visible estimated-distance row.

- [ ] **Step 2: Run the dashboard test and verify RED**

Run: `python -m pytest -q tests/test_web_compass.py`

Expected: FAIL because the page currently calls `signedAngle()` and still renders `distVal`.

- [ ] **Step 3: Implement unsigned display and remove the dashboard distance row**

In `main/web_page.html`, replace `signedAngle` with:

```javascript
function displayAngle(v){
    return norm360(v);
}
```

Use `displayAngle(d.angle)` for `aVal`. Remove `.dist` CSS, the distance HTML row, and the `distVal` SSE update. Do not modify `drawC()` or its pointer formula.

- [ ] **Step 4: Remove distance from the firmware and SSE contract**

In `main/i2s_es7210_record_example.c`, remove the distance header, static estimator, RMS aggregation and estimator update, initialization, and the two distance arguments passed to `wifi_server_push_data`.

Change the declaration and definition to:

```c
void wifi_server_push_data(float angle, float bird_conf,
                           float noise_conf, bool bird_detected);
```

Format SSE JSON as:

```c
"data: {\"angle\":%.1f,\"bird\":%.4f,\"noise\":%.4f,\"detected\":%s}\n\n"
```

Remove `distance_estimator.c` from `main/CMakeLists.txt`, then delete the estimator files and their obsolete tests.

- [ ] **Step 5: Run focused tests and reference checks**

Run:

```powershell
python -m pytest -q tests/test_web_compass.py
rg -n "distance_estimator|distance_array_rms|distance_m|distance_valid|distVal|signedAngle" main tests
```

Expected: dashboard tests pass; `rg` returns no matches.

- [ ] **Step 6: Commit the web/SSE cleanup**

```powershell
git add -A -- main tests
git commit -m "remove unreliable distance estimate"
```

### Task 3: Three-Frame Single-Pixel LED Rendering

**Files:**
- Create: `tests/fakes/esp_err.h`
- Create: `tests/fakes/esp_log.h`
- Create: `tests/fakes/led_strip.h`
- Create: `tests/test_led_indicator_main.c`
- Create: `tests/test_led_indicator.py`
- Modify: `main/led_indicator.c`

**Interfaces:**
- Consumes: `led_update_direction(float angle_deg, bool source_active)`
- Produces: two refreshes per rendered direction (all-off, then exactly one pixel) and a switch only on the third consecutive different-pixel request

- [ ] **Step 1: Add the fake LED driver and failing behavior test**

The fake `led_strip` API owns a ten-pixel bit mask. `led_strip_clear()` sets the working mask to zero, `led_strip_set_pixel()` sets one bit, and `led_strip_refresh()` appends the current mask to a refresh-history array.

The C test initializes the indicator, clears fake history, and verifies:

```c
led_update_direction(22.5f, true);
expect_refresh_pair(0, 1U << 0);

led_update_direction(337.5f, true);
expect_refresh_pair(0, 1U << 0);
led_update_direction(337.5f, true);
expect_refresh_pair(0, 1U << 0);
led_update_direction(337.5f, true);
expect_refresh_pair(0, 1U << 1);
```

Every non-zero refresh mask must contain exactly one bit. Compile the real `main/led_indicator.c` and `main/direction_mapping.c` against only the fake external driver headers.

- [ ] **Step 2: Run the focused LED test and verify RED**

Run: `python -m pytest -q tests/test_led_indicator.py`

Expected: FAIL because current rendering sends only the final pixel frame and switches on the second request.

- [ ] **Step 3: Implement explicit clear-refresh and three-frame confirmation**

Set `LED_SWITCH_CONFIRM_FRAMES` to `3U`. Add a private renderer that performs:

```c
led_strip_clear(s_led_strip);
led_strip_refresh(s_led_strip);
led_strip_set_pixel(s_led_strip, pixel, 0, LED_DIRECTION_BRIGHTNESS, 0);
led_strip_refresh(s_led_strip);
```

Use it for the held and selected pixel paths. Keep `led_all_off()` as one clear refresh. Preserve error logging and only update `s_last_pixel` after the complete selected-pixel sequence succeeds.

- [ ] **Step 4: Run the focused LED test and verify GREEN**

Run: `python -m pytest -q tests/test_led_indicator.py`

Expected: `1 passed` and all captured non-zero frames contain one pixel.

- [ ] **Step 5: Commit the LED behavior**

```powershell
git add -- main/led_indicator.c tests/fakes tests/test_led_indicator_main.c tests/test_led_indicator.py
git commit -m "latch one direction led at a time"
```

### Task 4: Regression and Firmware Verification

**Files:**
- Verify only: all modified files

**Interfaces:**
- Consumes: completed Tasks 1-3
- Produces: test and build evidence for handoff

- [ ] **Step 1: Run all host-side tests**

Run: `python -m pytest -q tests`

Expected: all tests pass, including bird decision, direction mapping, DOA tracker, SRP-PHAT, web compass, and LED output.

- [ ] **Step 2: Check source references and formatting**

Run:

```powershell
rg -n "distance_estimator|distance_array_rms|distance_m|distance_valid|distVal|signedAngle" main tests
git diff --check HEAD~3..HEAD
git status --short
```

Expected: no removed-distance matches, no whitespace errors, and a clean worktree.

- [ ] **Step 3: Run the ESP-IDF build when the environment is available**

Run: `idf.py build`

Expected: firmware links successfully. If `idf.py` is unavailable in the current PowerShell environment, report that exact limitation and do not substitute an unrelated ESP-IDF installation.

- [ ] **Step 4: Review the final commit range**

Run:

```powershell
git log --oneline 82b98e2..HEAD
git diff --stat 82b98e2..HEAD
```

Expected: only the approved bird decision, unsigned angle, distance removal, LED output, and their tests are present.
