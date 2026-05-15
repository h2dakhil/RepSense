/**
 * @file    imu.cpp
 * @brief   LSM6DSV (ST 6-DoF IMU) driver over I2C.
 *
 * The LSM6DSV is configured for:
 *   - Accelerometer: ±8 g, 208 Hz ODR, high-performance mode.
 *   - Gyroscope:     ±2000 dps, 208 Hz ODR, high-performance mode.
 *   - INT1: wake-on-motion (activity interrupt), routed to nRF52 P1.06.
 *
 * Motion-detect path: when the wrist is still the IMU's internal slope filter
 * keeps the activity bit low, INT1 stays low, MCU stays asleep at ~25 µA
 * (IMU current in low-power activity-detect mode). On motion the activity
 * engine drives INT1 high, the MCU wakes, and we reconfigure the device for
 * high-performance sampling.
 */

#include "imu.h"
#include "config.h"
#include "power.h"
#include <Wire.h>

namespace imu {

// ---------------------------------------------------------------------------
// LSM6DSV register map (subset used here)
// ---------------------------------------------------------------------------
constexpr uint8_t REG_FUNC_CFG_ACCESS = 0x01;
constexpr uint8_t REG_WHO_AM_I        = 0x0F;
constexpr uint8_t REG_CTRL1_XL        = 0x10;  ///< accel ODR / range
constexpr uint8_t REG_CTRL2_G         = 0x11;  ///< gyro  ODR / range
constexpr uint8_t REG_CTRL3_C         = 0x12;  ///< BDU, auto-inc
constexpr uint8_t REG_CTRL6_C         = 0x15;  ///< gyro perf mode
constexpr uint8_t REG_CTRL7_XL        = 0x16;  ///< accel perf mode
constexpr uint8_t REG_TAP_CFG0        = 0x56;  ///< latched / unlatched ints
constexpr uint8_t REG_WAKE_UP_THS     = 0x5B;
constexpr uint8_t REG_WAKE_UP_DUR     = 0x5C;
constexpr uint8_t REG_MD1_CFG         = 0x5E;  ///< route wake to INT1
constexpr uint8_t REG_OUTX_L_G        = 0x22;  ///< first of 12 data bytes

constexpr uint8_t WHO_AM_I_EXPECTED   = 0x70;

// Encoded CTRL register values (datasheet table 53/54):
//   CTRL1_XL = 0x58 -> ODR_XL=208 Hz (0101), FS_XL=±8 g (10), HP filter off.
//   CTRL2_G  = 0x5C -> ODR_G =208 Hz (0101), FS_G =±2000 dps (1100b nibble).
constexpr uint8_t CTRL1_XL_VAL = 0x58;
constexpr uint8_t CTRL2_G_VAL  = 0x5C;
constexpr uint8_t CTRL3_C_VAL  = 0x44;  ///< BDU=1, IF_INC=1
constexpr uint8_t WAKE_UP_THS_VAL = 0x02;  ///< ~63 mg per LSB × 2 = ~126 mg
constexpr uint8_t WAKE_UP_DUR_VAL = 0x00;  ///< minimum duration
constexpr uint8_t MD1_CFG_WAKE    = 0x20;  ///< INT1_WU bit

// ---------------------------------------------------------------------------
static volatile bool s_motionFlag = false;

/// @brief Low-level I2C single-byte write. Returns true on ACK.
static bool i2cWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

/// @brief Low-level I2C single-byte read.
static bool i2cRead(uint8_t reg, uint8_t &out) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(IMU_ADDR, (uint8_t)1) != 1) return false;
    out = Wire.read();
    return true;
}

/// @brief Burst read N bytes starting at reg (uses auto-increment).
static bool i2cReadBurst(uint8_t reg, uint8_t *buf, uint8_t n) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(IMU_ADDR, n) != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = Wire.read();
    return true;
}

/// @brief INT1 hardware ISR — fires on rising edge from IMU.
static void int1Isr() {
    s_motionFlag = true;
    power::onWakeInterrupt();
}

// ---------------------------------------------------------------------------

bool begin() {
    Wire.begin();
    Wire.setClock(I2C_FREQ);

    uint8_t who = 0;
    if (!i2cRead(REG_WHO_AM_I, who) || who != WHO_AM_I_EXPECTED) {
#ifdef DEBUG
        Serial.print("[imu] WHO_AM_I fail: 0x"); Serial.println(who, HEX);
#endif
        return false;
    }

    // Core config: enable block-data-update and register auto-increment so
    // multi-byte burst reads stay coherent across the LSB/MSB pair.
    if (!i2cWrite(REG_CTRL3_C, CTRL3_C_VAL)) return false;

    // Accel + gyro full-scale and ODR.
    if (!i2cWrite(REG_CTRL1_XL, CTRL1_XL_VAL)) return false;
    if (!i2cWrite(REG_CTRL2_G,  CTRL2_G_VAL))  return false;

    // High-performance mode is the reset default for CTRL6_C / CTRL7_XL on
    // the LSM6DSV — explicitly clear the low-power bits in case the part
    // was left in a different state by a prior boot.
    if (!i2cWrite(REG_CTRL6_C,  0x00)) return false;
    if (!i2cWrite(REG_CTRL7_XL, 0x00)) return false;

    pinMode(IMU_INT1, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT1), int1Isr, RISING);

    return true;
}

// ---------------------------------------------------------------------------

bool readIMU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz) {
    uint8_t buf[12];
    if (!i2cReadBurst(REG_OUTX_L_G, buf, 12)) {
#ifdef DEBUG
        Serial.println("[imu] burst read failed");
#endif
        return false;
    }

    // Gyro first (OUTX_L_G..OUTZ_H_G), then accel (OUTX_L_A..OUTZ_H_A).
    int16_t rgx = (int16_t)(buf[0]  | (buf[1]  << 8));
    int16_t rgy = (int16_t)(buf[2]  | (buf[3]  << 8));
    int16_t rgz = (int16_t)(buf[4]  | (buf[5]  << 8));
    int16_t rax = (int16_t)(buf[6]  | (buf[7]  << 8));
    int16_t ray = (int16_t)(buf[8]  | (buf[9]  << 8));
    int16_t raz = (int16_t)(buf[10] | (buf[11] << 8));

    gx = rgx * GYRO_SENS_2000;
    gy = rgy * GYRO_SENS_2000;
    gz = rgz * GYRO_SENS_2000;
    ax = rax * ACCEL_SENS_8G;
    ay = ray * ACCEL_SENS_8G;
    az = raz * ACCEL_SENS_8G;
    return true;
}

Sample read() {
    Sample s{};
    s.t_us = micros();
    s.ok = readIMU(s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
    return s;
}

// ---------------------------------------------------------------------------

void enableMotionWake() {
    // Wake-on-motion: threshold ~126 mg, duration = 0 (single sample over
    // threshold triggers). Route the activity bit to INT1 via MD1_CFG.
    i2cWrite(REG_WAKE_UP_THS, WAKE_UP_THS_VAL);
    i2cWrite(REG_WAKE_UP_DUR, WAKE_UP_DUR_VAL);

    // Latch interrupts so a short pulse isn't missed by the slow GPIOTE path.
    i2cWrite(REG_TAP_CFG0, 0x01);   // LIR=1
    i2cWrite(REG_MD1_CFG,  MD1_CFG_WAKE);

    s_motionFlag = false;
}

void disableMotionWake() {
    i2cWrite(REG_MD1_CFG, 0x00);
    s_motionFlag = false;
}

bool motionDetected()  { return s_motionFlag; }
void clearMotionFlag() { s_motionFlag = false; }

} // namespace imu
