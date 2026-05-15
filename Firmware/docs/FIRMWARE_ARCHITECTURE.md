# RepSense Firmware Architecture

Firmware for a wrist-mounted fitness tracker built on the Nordic nRF52840 with an
ST LSM6DSV 6-DoF IMU and a 0.91" SSD1306 OLED. The firmware sleeps under 35 µA,
wakes on motion, fuses IMU data through a quaternion EKF, counts reps via
peak/zero-crossing detection on the dominant axis, and classifies the exercise
with a heuristic decision tree.

---

## Module map

```
┌──────────────────────────────────────────────────────────────────┐
│                            main.cpp                              │
│      BOOT → SLEEP → ACTIVE → DISPLAY_UPDATE → SLEEP              │
└───────┬──────────────┬─────────────┬────────────┬────────────────┘
        │              │             │            │
   ┌────▼────┐    ┌────▼────┐   ┌────▼─────┐  ┌───▼──────┐
   │  power  │    │   imu   │   │   ekf    │  │ battery  │
   └─────────┘    └────┬────┘   └────┬─────┘  └──────────┘
                       │             │
                       ▼             ▼
                  ┌────────────────────────┐
                  │     rep_detector       │
                  └───────────┬────────────┘
                              ▼
                  ┌────────────────────────┐
                  │ exercise_classifier    │
                  └───────────┬────────────┘
                              ▼
                          ┌────────┐
                          │display │
                          └────────┘
```

---

## Module reference

### `config.h`
Single source of truth for pin assignments, I2C addresses, ADC scaling,
EKF noise covariances, rep-detector thresholds, per-exercise classifier
thresholds, and state-machine timing. No magic numbers may appear in
the `.cpp` files; everything is a named constant here.

### `power.h / power.cpp`
- **Responsibilities:** enable nRF52840 DC-DC converter (`NRF_POWER->DCDCEN = 1`),
  disable unused peripherals (UART0/UARTE0, SPIM0..2, PWM0..2) to hit the
  <35 µA sleep budget, manage System ON sleep via `__WFE`, and latch the
  IMU wake interrupt for the state machine.
- **Inputs:** wake ISR notifications from `imu`.
- **Outputs:** `wakeRequested()` flag consumed by the main loop.
- **Connections:** called by `main.cpp` on boot and at sleep/wake transitions.

### `imu.h / imu.cpp`
- **Responsibilities:** LSM6DSV init over I2C @ 400 kHz, WHO_AM_I verification
  (expects `0x70`), accel `±8 g` / gyro `±2000 dps` at 208 Hz ODR in
  high-performance mode, motion-detect wake configuration via
  `WAKE_UP_THS` / `WAKE_UP_DUR` / `MD1_CFG`, INT1 routed to P1.06 with a
  rising-edge attachInterrupt.
- **Inputs:** I2C bus, INT1 pin.
- **Outputs:** `imu::read()` returns a 6-axis SI-units sample plus an `ok` flag.
- **Connections:** consumed by `ekf` and `rep_detector` indirectly via `main.cpp`.
  Bus errors are logged under `#ifdef DEBUG` and degraded gracefully — the EKF
  step is skipped for that tick.

### `ekf.h / ekf.cpp`
- **Responsibilities:** quaternion-based EKF, the core algorithm.
  - **State:** `q = [q0,q1,q2,q3]` (body → world unit quaternion).
  - **Predict:** `F = I + 0.5·Ω(ω)·dt`, `q⁻ = F·q`, `P⁻ = F·P·Fᵀ + Q`.
  - **Update:** measurement `h(q)` is the predicted gravity direction in
    body coordinates; analytical 3×4 Jacobian `H = ∂h/∂q`; innovation
    `y = ẑ − h(q)`; Kalman gain `K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹`;
    `q⁺ = q + K·y` (re-normalised); `P⁺ = (I − K·H)·P`.
  - **Adaptive R:** `R` inflates with `(|a| − g)²` to down-weight the accel
    update during transient limb motion.
  - **Outputs:** `getRoll/Pitch/Yaw` in degrees, `getQuaternion`, and a
    gravity-removed linear acceleration vector for the rep detector.
- **Tunables:** `EKF_Q_GYRO`, `EKF_R_ACCEL`, `EKF_P_INIT` in `config.h`.
- **Notes:** all single-precision (`float`) — Cortex-M4F has a hardware FPU
  for single-precision only; using `double` would silently fall back to
  software emulation and miss the 200 Hz deadline.

### `rep_detector.h / rep_detector.cpp`
- **Responsibilities:** turn streaming EKF output into discrete reps.
  - Rolling 64-sample window of pitch/roll/yaw for dominant-axis RMS selection.
  - First-order high-pass filter (`y = α·(y_prev + x − x_prev)`) removes the
    user's neutral-wrist DC offset so signals oscillate about zero.
  - Zero-crossing state machine with hysteresis (`REP_ZERO_HYSTERESIS`):
    a `Positive → Negative → Positive` traversal commits one rep.
  - Per-rep validation: duration ∈ [0.4 s, 6 s] and ROM ≥ 25°.
  - On commit, snapshots `RepFeatures` (ROM, peak |ω|, peak |a|, RMS
    per axis, duration, timestamp).
- **Inputs:** Euler angles + gyro + linear-accel from `ekf`.
- **Outputs:** `getRepCount`, `getLastRepFeatures`, and the `process()`
  return flag.

### `exercise_classifier.h / exercise_classifier.cpp`
- **Responsibilities:** heuristic decision tree mapping `RepFeatures` →
  `ExerciseType`. Ordered most-unique signature first:
  Deadlift → Squat → Lateral Raise → Row → Overhead Press → Shoulder Press →
  Bicep Curl → Bench Press → Tricep Pushdown.
- **Confidence:** requires `CLASSIFIER_CONFIRM_REPS` (= 3) consecutive
  matching reps before committing a label. Single-rep noise can't latch
  the wrong exercise on the display.
- **Inputs:** `RepFeatures` struct from `rep_detector`.
- **Outputs:** `Result { ExerciseType, bool confident }` and a stable
  `getCommitted()` accessor for the display.

### `battery.h / battery.cpp`
- **Responsibilities:** read AIN via the SAADC (`AR_VDD4` reference, 12-bit
  resolution), convert via `VBAT = ADC × 6.6 / 4096`, then map voltage →
  percent through a piecewise-linear interpolation of an 8-point LiPo
  discharge curve. Sampled at most once per 30 s.
- **Inputs:** AIN voltage at the battery divider.
- **Outputs:** `getBatteryVoltage()`, `getBatteryPercent()`.

### `display.h / display.cpp`
- **Responsibilities:** SSD1306 driver wrapping Adafruit_GFX/SSD1306.
  - **Boot screen:** project name + version for `BOOT_SCREEN_MS`.
  - **Active screen:** exercise name (top-left), rep count (centre,
    size 3), battery icon (top-right). Partial redraws — each region is
    only repainted when its underlying value changes.
  - **Rest screen:** "REST" + last set summary.
  - **Sleep:** `SSD1306_DISPLAYOFF` to drop panel current to standby.
- **Inputs:** committed exercise label, rep count, battery percent.
- **Outputs:** I2C traffic to address `0x3C`.

### `main.cpp`
- **Responsibilities:** the state machine.
  - **BOOT:** init power → display → IMU → battery → EKF → detector →
    classifier; show splash; arm wake; → SLEEP.
  - **SLEEP:** WFE until INT1; clear flags; → ACTIVE.
  - **ACTIVE:** 200 Hz paced loop; IMU read → EKF step → rep_detector →
    classifier; partial repaint on commit. If `IDLE_BEFORE_SLEEP_MS`
    elapses without motion → SLEEP.
  - **DISPLAY_UPDATE:** single-shot OLED repaint after a confirmed rep,
    then back to ACTIVE.

---

## Data flow (one active tick)

```
IMU.read()  → 6 floats (a, ω)
   │
   ▼
ekf::step()   → q, roll/pitch/yaw, linAccel
   │
   ▼
rep_detector::process()  → bool repCommitted, RepFeatures (on commit)
   │ (if commit)
   ▼
classifier::classify()   → Result { type, confident }
   │
   ▼
display::showActive(type, repCount, batteryPct)
```

## Power budget (target)

| Component                | Current      |
|--------------------------|--------------|
| nRF52840 System ON sleep | ~1.5 µA      |
| LSM6DSV activity-detect  | ~25 µA       |
| SSD1306 panel off        | ~5 µA        |
| **Total sleep**          | **< 35 µA**  |

## Build

```
pio run -e adafruit_feather_nrf52840
pio run -e adafruit_feather_nrf52840 -t upload
pio device monitor
```

Release builds: drop `-DDEBUG` from `platformio.ini` to compile out all
Serial logging and avoid the UART power cost.
