/**
 * @file    imu.h
 * @brief   LSM6DSV driver — init, raw read, motion-detect interrupt config.
 */

#ifndef REPSENSE_IMU_H
#define REPSENSE_IMU_H

#include <Arduino.h>

namespace imu {

struct Sample {
    float ax, ay, az;  ///< m/s²
    float gx, gy, gz;  ///< rad/s
    uint32_t t_us;     ///< micros() at read
    bool ok;
};

/// @brief Probe WHO_AM_I, configure accel/gyro/INT1. Returns false on bus error.
bool begin();

/// @brief Read all 6 axes and convert to SI units. Caller passes references.
bool readIMU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz);

/// @brief Convenience wrapper that returns a Sample struct.
Sample read();

/// @brief Arm the LSM6DSV wake-on-motion engine and attach the INT1 ISR.
void enableMotionWake();

/// @brief Disarm wake-on-motion (active sampling mode).
void disableMotionWake();

/// @brief Returns true if INT1 has fired since last clearMotionFlag().
bool motionDetected();
void clearMotionFlag();

} // namespace imu

#endif // REPSENSE_IMU_H
