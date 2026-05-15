/**
 * @file    ekf.h
 * @brief   Quaternion-based Extended Kalman Filter for 6-DoF IMU fusion.
 *
 * State : x = [q0 q1 q2 q3]^T   (unit quaternion, body -> world)
 * Inputs: gyro ω (rad/s), accel a (m/s²)
 * Output: roll/pitch/yaw (deg), gravity-removed linear acceleration (m/s²).
 *
 * Yaw is unobservable from accel alone, so getYaw() drifts slowly with gyro
 * bias — this is acceptable for rep counting, where pitch and roll are the
 * load-bearing signals.
 */

#ifndef REPSENSE_EKF_H
#define REPSENSE_EKF_H

#include <Arduino.h>

namespace ekf {

struct Vec3 { float x, y, z; };

/// @brief Reset filter to identity quaternion and initial covariance.
void reset();

/// @brief Run one full predict + update cycle. dt in seconds.
void step(float gx, float gy, float gz,
          float ax, float ay, float az,
          float dt);

float getRoll();    ///< deg, rotation about X (body)
float getPitch();   ///< deg, rotation about Y (body)
float getYaw();     ///< deg, rotation about Z (body)

/// @brief Copy the current quaternion into out[4].
void getQuaternion(float out[4]);

/// @brief World-frame linear acceleration (gravity removed), m/s².
Vec3 getLinearAccel();

} // namespace ekf

#endif // REPSENSE_EKF_H
