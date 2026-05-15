/**
 * @file    rep_detector.cpp
 * @brief   Rep counting via high-pass-filtered zero-crossing detection.
 *
 * Algorithm
 * ---------
 *  1. Pick a dominant axis from a rolling-window RMS of pitch / roll / yaw.
 *  2. High-pass-filter that axis to remove the DC offset (the user's neutral
 *     wrist orientation) so we get a zero-mean oscillation:
 *        y[n] = α·(y[n-1] + x[n] - x[n-1]),    α = REP_HPF_ALPHA.
 *  3. Watch for sign changes with hysteresis (REP_ZERO_HYSTERESIS).  A
 *     descend-then-ascend pair = one rep.
 *  4. Validate against timing window (400 ms – 6 s) and minimum ROM (25°).
 *  5. On commit, snapshot per-rep features (ROM, peak ω, peak |a|, RMS per
 *     axis) and hand them to the classifier.
 *
 * The rolling window holds REP_WINDOW_SAMPLES of Euler angles for both axis
 * selection and ROM measurement. At 200 Hz a 64-sample window = 320 ms,
 * long enough to capture the bottom-out of any rep we care about.
 */

#include "rep_detector.h"
#include "config.h"
#include <math.h>

namespace rep_detector {

// ---------------------------------------------------------------------------
// Rolling buffer (circular). Stores raw Euler angles in degrees.
// ---------------------------------------------------------------------------
static float    bufPitch[REP_WINDOW_SAMPLES];
static float    bufRoll [REP_WINDOW_SAMPLES];
static float    bufYaw  [REP_WINDOW_SAMPLES];
static uint16_t bufIdx   = 0;
static uint16_t bufCount = 0;

// High-pass filter state (one filter per axis).
static float hpfPrevInPitch = 0.0f, hpfOutPitch = 0.0f;
static float hpfPrevInRoll  = 0.0f, hpfOutRoll  = 0.0f;
static float hpfPrevInYaw   = 0.0f, hpfOutYaw   = 0.0f;

// Zero-crossing state machine.
enum class Phase : uint8_t { Idle, Negative, Positive };
static Phase    phase = Phase::Idle;
static uint32_t repStartMs = 0;
static float    repMin = 0.0f, repMax = 0.0f;
static float    repPeakOmega = 0.0f;
static float    repPeakAccel = 0.0f;

// Committed rep state.
static uint32_t    s_repCount = 0;
static RepFeatures s_lastFeatures = {};

// ---------------------------------------------------------------------------
static inline void pushBuf(float p, float r, float y) {
    bufPitch[bufIdx] = p;
    bufRoll [bufIdx] = r;
    bufYaw  [bufIdx] = y;
    bufIdx = (bufIdx + 1) % REP_WINDOW_SAMPLES;
    if (bufCount < REP_WINDOW_SAMPLES) ++bufCount;
}

/// @brief RMS of (x − mean(x)) over the active portion of the buffer.
static float windowRMS(const float *buf) {
    if (bufCount == 0) return 0.0f;
    float mean = 0.0f;
    for (uint16_t i = 0; i < bufCount; ++i) mean += buf[i];
    mean /= bufCount;
    float sq = 0.0f;
    for (uint16_t i = 0; i < bufCount; ++i) {
        const float d = buf[i] - mean;
        sq += d * d;
    }
    return sqrtf(sq / bufCount);
}

static DominantAxis chooseDominantAxis(float &rmsP, float &rmsR, float &rmsY) {
    rmsP = windowRMS(bufPitch);
    rmsR = windowRMS(bufRoll);
    rmsY = windowRMS(bufYaw);
    if (rmsP >= rmsR && rmsP >= rmsY) return DominantAxis::Pitch;
    if (rmsR >= rmsP && rmsR >= rmsY) return DominantAxis::Roll;
    return DominantAxis::Yaw;
}

// ---------------------------------------------------------------------------
// High-pass filter: y[n] = α·(y[n-1] + x[n] - x[n-1])
// ---------------------------------------------------------------------------
static inline float hpf(float x, float &prevIn, float &prevOut) {
    const float y = REP_HPF_ALPHA * (prevOut + x - prevIn);
    prevIn  = x;
    prevOut = y;
    return y;
}

// ---------------------------------------------------------------------------

void reset() {
    bufIdx = 0; bufCount = 0;
    hpfPrevInPitch = hpfOutPitch = 0.0f;
    hpfPrevInRoll  = hpfOutRoll  = 0.0f;
    hpfPrevInYaw   = hpfOutYaw   = 0.0f;
    phase = Phase::Idle;
    repMin = repMax = 0.0f;
    repPeakOmega = repPeakAccel = 0.0f;
    s_repCount = 0;
    s_lastFeatures = {};
}

uint32_t getRepCount() { return s_repCount; }

void resetReps() {
    s_repCount = 0;
    s_lastFeatures = {};
    phase = Phase::Idle;
}

const RepFeatures &getLastRepFeatures() { return s_lastFeatures; }

// ---------------------------------------------------------------------------
// Main entry point — call at the EKF rate (200 Hz).
// ---------------------------------------------------------------------------
bool process(float roll, float pitch, float yaw,
             float gx_rad, float gy_rad, float gz_rad,
             float linAccelZ) {

    pushBuf(pitch, roll, yaw);

    // 1) High-pass each axis so we have zero-mean signals.
    const float hp_p = hpf(pitch, hpfPrevInPitch, hpfOutPitch);
    const float hp_r = hpf(roll,  hpfPrevInRoll,  hpfOutRoll);
    const float hp_y = hpf(yaw,   hpfPrevInYaw,   hpfOutYaw);

    // 2) Pick the dominant axis from the buffer's RMS. We do this every
    //    sample — cheap (3 small loops), and it lets the dominant axis
    //    follow the user if they switch exercises mid-session.
    float rmsP, rmsR, rmsY;
    const DominantAxis axis = chooseDominantAxis(rmsP, rmsR, rmsY);
    float signal = 0.0f;
    float omegaDegOnAxis = 0.0f;
    switch (axis) {
        case DominantAxis::Pitch: signal = hp_p; omegaDegOnAxis = gy_rad * (180.0f / PI); break;
        case DominantAxis::Roll:  signal = hp_r; omegaDegOnAxis = gx_rad * (180.0f / PI); break;
        case DominantAxis::Yaw:   signal = hp_y; omegaDegOnAxis = gz_rad * (180.0f / PI); break;
    }

    // 3) Zero-crossing state machine with hysteresis.
    bool repCommitted = false;
    const uint32_t now = millis();

    // Track instantaneous extrema for ROM measurement during an in-flight rep.
    if (phase != Phase::Idle) {
        if (signal > repMax) repMax = signal;
        if (signal < repMin) repMin = signal;
        const float w = fabsf(omegaDegOnAxis);
        if (w > repPeakOmega) repPeakOmega = w;
        const float a = fabsf(linAccelZ);
        if (a > repPeakAccel) repPeakAccel = a;
    }

    switch (phase) {
        case Phase::Idle:
            // Wait for the signal to swing past +peak-threshold OR -peak-threshold;
            // either direction can start a rep depending on which way the wrist
            // moves first. Establish initial phase from that direction.
            if (signal >  REP_PEAK_THRESH_DEG) {
                phase = Phase::Positive;
                repStartMs = now;
                repMin = signal; repMax = signal;
                repPeakOmega = fabsf(omegaDegOnAxis);
                repPeakAccel = fabsf(linAccelZ);
            } else if (signal < -REP_PEAK_THRESH_DEG) {
                phase = Phase::Negative;
                repStartMs = now;
                repMin = signal; repMax = signal;
                repPeakOmega = fabsf(omegaDegOnAxis);
                repPeakAccel = fabsf(linAccelZ);
            }
            break;

        case Phase::Positive:
            // Wait for signal to cross to the negative side (past hysteresis).
            if (signal < -REP_ZERO_HYSTERESIS) {
                phase = Phase::Negative;
            }
            break;

        case Phase::Negative:
            // Returning past +hyst closes the rep (down → up cycle complete).
            if (signal > REP_ZERO_HYSTERESIS) {
                const uint32_t dur = now - repStartMs;
                const float    rom = repMax - repMin;

                // 4) Validation.
                const bool durationOK = (dur >= REP_MIN_DURATION_MS &&
                                         dur <= REP_MAX_DURATION_MS);
                const bool romOK      = (rom >= REP_MIN_ROM_DEG);

                if (durationOK && romOK) {
                    s_repCount++;
                    s_lastFeatures.dominantAxis      = axis;
                    s_lastFeatures.rangeOfMotionDeg  = rom;
                    s_lastFeatures.peakAngularVelDeg = repPeakOmega;
                    s_lastFeatures.peakLinearAccel   = repPeakAccel;
                    s_lastFeatures.rmsPitch          = rmsP;
                    s_lastFeatures.rmsRoll           = rmsR;
                    s_lastFeatures.rmsYaw            = rmsY;
                    s_lastFeatures.durationMs        = dur;
                    s_lastFeatures.timestampMs       = now;
                    repCommitted = true;
                }
                // Reset for the next rep. Restart in Positive — we just
                // crossed past the +hysteresis boundary.
                phase = Phase::Positive;
                repStartMs = now;
                repMin = signal; repMax = signal;
                repPeakOmega = fabsf(omegaDegOnAxis);
                repPeakAccel = fabsf(linAccelZ);
            }
            break;
    }

    return repCommitted;
}

} // namespace rep_detector
