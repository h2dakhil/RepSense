/**
 * @file    config.h
 * @brief   Compile-time configuration for the RepSense fitness tracker firmware.
 *
 * All pin assignments, I2C addresses, sample rates, EKF noise covariances,
 * and rep-detector / classifier thresholds live here. Source modules must
 * never embed magic numbers — pull from this file instead.
 *
 * Target hardware: nRF52840 (Adafruit Feather pinout) + LSM6DSV IMU +
 *                  SSD1306 0.91" OLED.
 */

#ifndef REPSENSE_CONFIG_H
#define REPSENSE_CONFIG_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Firmware identity
// ---------------------------------------------------------------------------
#define FIRMWARE_NAME       "RepSense"
#define FIRMWARE_VERSION    "0.1.0"

// ---------------------------------------------------------------------------
// I2C bus
// ---------------------------------------------------------------------------
// Arduino-nRF52 maps Pin = (port * 32) + bit. P1.04 -> 36, P1.05 -> 37, P1.06 -> 38.
constexpr uint8_t  IMU_SDA          = 36;        ///< P1.04
constexpr uint8_t  IMU_SCL          = 37;        ///< P1.05
constexpr uint8_t  IMU_INT1         = 38;        ///< P1.06
constexpr uint32_t I2C_FREQ         = 400000UL;  ///< Fast-mode I2C (400 kHz)

// ---------------------------------------------------------------------------
// Device addresses
// ---------------------------------------------------------------------------
// LSM6DSV: SA0=GND -> 7-bit 0x6A. SSD1306: 7-bit 0x3C (0x78 with R/W bit).
constexpr uint8_t IMU_ADDR          = 0x6A;
constexpr uint8_t DISPLAY_ADDR_7BIT = 0x3C;
constexpr uint8_t DISPLAY_ADDR      = 0x78;      ///< 8-bit form referenced in spec

// ---------------------------------------------------------------------------
// Battery monitor
// ---------------------------------------------------------------------------
// 1 MΩ / 1 MΩ divider feeds VBAT/2 to AIN. With internal VDD reference (3.3 V)
// and 12-bit ADC: VBAT = ADC × (2 × 3.3) / 4096 = ADC × 6.6 / 4096.
constexpr uint8_t  BATTERY_AIN          = A6;    ///< Feather VBAT pin (P0.05 / AIN3)
constexpr float    BATTERY_ADC_TO_VOLTS = 6.6f / 4096.0f;
constexpr uint32_t BATTERY_SAMPLE_MS    = 30000; ///< Sample every 30 s

// ---------------------------------------------------------------------------
// IMU sample / EKF rates
// ---------------------------------------------------------------------------
constexpr float    EKF_DT           = 0.005f;    ///< 200 Hz EKF tick (s)
constexpr uint32_t SAMPLE_PERIOD_US = 5000;      ///< 5 ms = 200 Hz

// LSM6DSV scale conversion (sensitivity at ±8 g and ±2000 dps).
constexpr float ACCEL_SENS_8G   = 0.244f * 0.001f * 9.80665f; ///< LSB -> m/s²
constexpr float GYRO_SENS_2000  = 70.0f * 0.001f * (PI / 180.0f); ///< LSB -> rad/s
constexpr float GRAVITY_MPS2    = 9.80665f;

// ---------------------------------------------------------------------------
// EKF noise covariances (tuned for wrist-worn IMU @ 200 Hz)
// ---------------------------------------------------------------------------
// Process noise (gyro integration) — small because gyro is low-noise.
constexpr float EKF_Q_GYRO      = 1.0e-5f;
// Measurement noise (accel gravity vector) — larger because wrist accel is
// contaminated by limb motion. The EKF down-weights the accel update
// dynamically when |a| ≠ g, but this is the baseline.
constexpr float EKF_R_ACCEL     = 5.0e-2f;
// Initial state covariance — large to allow fast convergence on first sample.
constexpr float EKF_P_INIT      = 1.0e-2f;

// ---------------------------------------------------------------------------
// Rep detector thresholds
// ---------------------------------------------------------------------------
constexpr uint16_t REP_WINDOW_SAMPLES   = 64;     ///< Rolling buffer length
constexpr float    REP_HPF_ALPHA        = 0.97f;  ///< High-pass filter (DC removal)
constexpr float    REP_PEAK_THRESH_DEG  = 15.0f;  ///< Min |signal| to arm peak detector
constexpr float    REP_ZERO_HYSTERESIS  = 3.0f;   ///< Deg of hysteresis around zero
constexpr uint32_t REP_MIN_DURATION_MS  = 400;    ///< Reject reps <0.4 s
constexpr uint32_t REP_MAX_DURATION_MS  = 6000;   ///< Reject reps >6.0 s
constexpr float    REP_MIN_ROM_DEG      = 25.0f;  ///< Min range-of-motion to count

// ---------------------------------------------------------------------------
// Exercise classifier — per-exercise thresholds
// ---------------------------------------------------------------------------
// (ROM in degrees, duration in ms, peak gyro in deg/s, peak linear accel in m/s²)

// BICEP_CURL
constexpr float BICEP_ROM_MIN = 80.0f, BICEP_ROM_MAX = 140.0f;
constexpr float BICEP_DUR_MIN = 1000.0f, BICEP_DUR_MAX = 3000.0f;
constexpr float BICEP_PEAK_OMEGA_MIN = 90.0f;

// SHOULDER_PRESS
constexpr float SHOULDER_ROM_MIN = 60.0f, SHOULDER_ROM_MAX = 110.0f;
constexpr float SHOULDER_DUR_MIN = 1500.0f, SHOULDER_DUR_MAX = 3500.0f;
constexpr float SHOULDER_VACCEL_MIN = 4.0f;

// BENCH_PRESS
constexpr float BENCH_ROM_MIN = 40.0f, BENCH_ROM_MAX = 80.0f;
constexpr float BENCH_DUR_MIN = 2000.0f, BENCH_DUR_MAX = 4000.0f;
constexpr float BENCH_ROLL_MAX = 20.0f;

// LATERAL_RAISE
constexpr float LATRAISE_ROM_MIN = 60.0f, LATRAISE_ROM_MAX = 100.0f;
constexpr float LATRAISE_DUR_MIN = 1500.0f, LATRAISE_DUR_MAX = 3000.0f;
constexpr float LATRAISE_PITCH_MAX = 25.0f;

// SQUAT
constexpr float SQUAT_ROM_MIN = 30.0f, SQUAT_ROM_MAX = 60.0f;
constexpr float SQUAT_DUR_MIN = 2500.0f, SQUAT_DUR_MAX = 5000.0f;
constexpr float SQUAT_PEAK_ACCEL_MIN = 5.0f;

// DEADLIFT
constexpr float DEADLIFT_ROM_MIN = 50.0f, DEADLIFT_ROM_MAX = 90.0f;
constexpr float DEADLIFT_DUR_MIN = 2000.0f, DEADLIFT_DUR_MAX = 4500.0f;
constexpr float DEADLIFT_PEAK_ACCEL_MIN = 7.0f;

// OVERHEAD_PRESS
constexpr float OHP_ROM_MIN = 80.0f, OHP_ROM_MAX = 130.0f;
constexpr float OHP_DUR_MIN = 1500.0f, OHP_DUR_MAX = 3500.0f;
constexpr float OHP_VACCEL_MIN = 4.0f;

// TRICEP_PUSHDOWN
constexpr float TRICEP_ROM_MIN = 50.0f, TRICEP_ROM_MAX = 90.0f;
constexpr float TRICEP_DUR_MIN = 800.0f, TRICEP_DUR_MAX = 2000.0f;
constexpr float TRICEP_ROLL_MAX = 15.0f;

// ROW
constexpr float ROW_ROM_MIN = 50.0f, ROW_ROM_MAX = 90.0f;
constexpr float ROW_DUR_MIN = 1500.0f, ROW_DUR_MAX = 3000.0f;
constexpr float ROW_COMBINED_RATIO_MIN = 0.4f;   ///< roll RMS / pitch RMS

// Number of consecutive matching reps required before classifier commits.
constexpr uint8_t CLASSIFIER_CONFIRM_REPS = 3;

// ---------------------------------------------------------------------------
// State-machine timing
// ---------------------------------------------------------------------------
constexpr uint32_t IDLE_BEFORE_SLEEP_MS = 5000;   ///< 5 s of no motion -> sleep
constexpr uint32_t BOOT_SCREEN_MS       = 1500;   ///< Splash duration

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
constexpr uint8_t  DISPLAY_WIDTH  = 128;
constexpr uint8_t  DISPLAY_HEIGHT = 32;

#endif // REPSENSE_CONFIG_H
