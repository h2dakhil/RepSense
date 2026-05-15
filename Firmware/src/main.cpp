/**
 * @file    main.cpp
 * @brief   Top-level state machine for the RepSense fitness tracker.
 *
 * States
 * ------
 *   BOOT             → init all modules, splash screen, arm wake interrupt.
 *   SLEEP            → System ON sleep, MCU at ~1.5 µA, IMU armed for wake.
 *   ACTIVE           → 200 Hz sampling, EKF, rep detect, classifier.
 *   DISPLAY_UPDATE   → triggered after a confirmed rep; partial OLED repaint.
 *
 * Transitions
 * -----------
 *   BOOT  → SLEEP                         once init done.
 *   SLEEP → ACTIVE                        on INT1 (motion-detect wake).
 *   ACTIVE → DISPLAY_UPDATE → ACTIVE      after each confirmed rep.
 *   ACTIVE → SLEEP                        after IDLE_BEFORE_SLEEP_MS
 *                                         (5 s) without motion.
 */

#include <Arduino.h>
#include "config.h"
#include "power.h"
#include "imu.h"
#include "ekf.h"
#include "rep_detector.h"
#include "exercise_classifier.h"
#include "battery.h"
#include "display.h"

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum class State : uint8_t { Boot, Sleep, Active, DisplayUpdate };

struct Machine {
    State    state            = State::Boot;
    uint32_t lastMotionMs     = 0;
    uint32_t lastSampleUs     = 0;
    uint32_t lastDisplayReps  = UINT32_MAX;
};
static Machine sm;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// @brief True if linear-accel magnitude exceeds a small motion threshold.
static bool sampleHasMotion(const ekf::Vec3 &la) {
    const float m2 = la.x*la.x + la.y*la.y + la.z*la.z;
    // ~0.5 m/s² noise floor at the wrist; below this we treat as "still".
    return m2 > (0.5f * 0.5f);
}

/// @brief Repaint the active screen with current state.
static void repaintActive() {
    display::showActive(exercise_classifier::getCommitted(),
                        (int)rep_detector::getRepCount(),
                        (int)battery::getBatteryPercent());
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
    // Don't block forever waiting for a host — production wrists rarely have one.
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 1500) {}
    Serial.println("[boot] RepSense firmware starting");
#endif

    // ----- Boot: init every module in dependency order. -----
    power::enableDCDC();
    power::configureLowPowerPeripherals();

    Wire.begin();
    Wire.setClock(I2C_FREQ);

    if (!display::begin()) {
#ifdef DEBUG
        Serial.println("[boot] display init failed");
#endif
    } else {
        display::showBootScreen();
    }

    if (!imu::begin()) {
#ifdef DEBUG
        Serial.println("[boot] imu init failed");
#endif
    }

    battery::begin();
    ekf::reset();
    rep_detector::reset();
    exercise_classifier::reset();

    // Arm IMU wake-on-motion, blank the OLED, and drop straight to sleep.
    imu::enableMotionWake();
    display::sleep();
    sm.state        = State::Sleep;
    sm.lastMotionMs = millis();
}

// ---------------------------------------------------------------------------

void loop() {
    switch (sm.state) {

    // ------------------------------------------------------------------
    case State::Sleep: {
        // Sleep until the IMU's wake interrupt fires. The CPU sits in WFE
        // at ~1.5 µA + IMU activity-detect (~25 µA) + display off (~5 µA).
        power::enterSleep();

        if (power::wakeRequested() || imu::motionDetected()) {
            power::clearWake();
            imu::clearMotionFlag();
            imu::disableMotionWake();      // switch IMU into high-perf mode
            power::restoreActivePeripherals();
            display::wake();

            // Reset filter state for a fresh session segment.
            ekf::reset();
            rep_detector::resetReps();
            exercise_classifier::reset();

            repaintActive();
            sm.state        = State::Active;
            sm.lastMotionMs = millis();
            sm.lastSampleUs = micros();
        }
        break;
    }

    // ------------------------------------------------------------------
    case State::Active: {
        // Tight 200 Hz sampling loop. We pace via micros() rather than
        // delay() so the CPU can drop to __WFE between samples for a few
        // dozen µs of leakage savings.
        const uint32_t now_us = micros();
        if ((now_us - sm.lastSampleUs) < SAMPLE_PERIOD_US) {
            __WFE();
            break;
        }
        sm.lastSampleUs += SAMPLE_PERIOD_US;

        imu::Sample s = imu::read();
        if (!s.ok) break;   // already logged in imu module under DEBUG

        ekf::step(s.gx, s.gy, s.gz, s.ax, s.ay, s.az, EKF_DT);

        const float pitch = ekf::getPitch();
        const float roll  = ekf::getRoll();
        const float yaw   = ekf::getYaw();
        const ekf::Vec3 la = ekf::getLinearAccel();

        const bool repCommitted = rep_detector::process(
            roll, pitch, yaw, s.gx, s.gy, s.gz, la.z);

        if (repCommitted) {
            (void)exercise_classifier::classify(
                rep_detector::getLastRepFeatures());
            sm.state = State::DisplayUpdate;  // fall-through next iteration
        }

        battery::update();   // throttled internally to BATTERY_SAMPLE_MS

        if (sampleHasMotion(la)) {
            sm.lastMotionMs = millis();
        } else if ((millis() - sm.lastMotionMs) >= IDLE_BEFORE_SLEEP_MS) {
            // No motion for 5 s — drop back to sleep.
            display::sleep();
            imu::enableMotionWake();
            sm.state = State::Sleep;
        }
        break;
    }

    // ------------------------------------------------------------------
    case State::DisplayUpdate: {
        // Single-shot OLED repaint after a confirmed rep, then back to ACTIVE.
        repaintActive();
        sm.state = State::Active;
        break;
    }

    // ------------------------------------------------------------------
    case State::Boot:
    default:
        // setup() handles Boot; reaching here means a bug — fall through to Sleep.
        sm.state = State::Sleep;
        break;
    }
}
