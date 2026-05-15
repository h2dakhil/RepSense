/**
 * @file    rep_detector.h
 * @brief   Peak / zero-crossing rep counter built on top of EKF outputs.
 */

#ifndef REPSENSE_REP_DETECTOR_H
#define REPSENSE_REP_DETECTOR_H

#include <Arduino.h>

namespace rep_detector {

enum class DominantAxis : uint8_t { Pitch, Roll, Yaw };

/// @brief Feature snapshot for the most-recent committed rep — fed to the classifier.
struct RepFeatures {
    DominantAxis dominantAxis;
    float        rangeOfMotionDeg;  ///< max - min of dominant axis over rep
    float        peakAngularVelDeg; ///< max |gyro on dominant axis|, deg/s
    float        peakLinearAccel;   ///< max |a_world.z| over rep, m/s²
    float        rmsPitch;          ///< RMS of high-passed pitch over rep
    float        rmsRoll;           ///< RMS of high-passed roll  over rep
    float        rmsYaw;            ///< RMS of high-passed yaw   over rep
    uint32_t     durationMs;
    uint32_t     timestampMs;       ///< millis() at rep commit
};

/// @brief Zero buffers, counters, and filter state.
void reset();

/// @brief Push one EKF sample into the detector. Returns true if a rep was just committed.
bool process(float roll, float pitch, float yaw,
             float gx_rad, float gy_rad, float gz_rad,
             float linAccelZ);

uint32_t getRepCount();
void     resetReps();

const RepFeatures &getLastRepFeatures();

} // namespace rep_detector

#endif // REPSENSE_REP_DETECTOR_H
