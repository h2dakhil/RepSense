/**
 * @file    exercise_classifier.cpp
 * @brief   Heuristic per-rep classifier.
 *
 * The decision tree is intentionally ordered most-unique → least-unique
 * signature: exercises with rare features (very high peak accel for
 * Deadlift, roll-dominant motion for Lateral Raise) are tested first so a
 * later "generic pitch" rule (Bicep Curl, Tricep Pushdown) doesn't steal
 * the rep.
 *
 * A streak counter requires CLASSIFIER_CONFIRM_REPS consecutive matching
 * classifications before promoting a label to "committed" — this prevents
 * a single noisy rep at the start of a set from latching the wrong label.
 */

#include "exercise_classifier.h"
#include "config.h"
#include <math.h>

namespace exercise_classifier {

using rep_detector::DominantAxis;
using rep_detector::RepFeatures;

static ExerciseType s_committed   = ExerciseType::Unknown;
static ExerciseType s_streakType  = ExerciseType::Unknown;
static uint8_t      s_streakCount = 0;

// ---------------------------------------------------------------------------
// Per-rep classifier — pure function of features. Returns Unknown if no
// exercise's thresholds are satisfied.
// ---------------------------------------------------------------------------
static ExerciseType classifyRaw(const RepFeatures &f) {
    const float rom   = f.rangeOfMotionDeg;
    const float dur   = (float)f.durationMs;
    const float peakA = f.peakLinearAccel;
    const float peakW = f.peakAngularVelDeg;
    const bool  pitchDom = (f.dominantAxis == DominantAxis::Pitch);
    const bool  rollDom  = (f.dominantAxis == DominantAxis::Roll);

    // 1) DEADLIFT — uniquely high peak accel + hip-hinge pitch.
    if (pitchDom &&
        peakA >= DEADLIFT_PEAK_ACCEL_MIN &&
        rom   >= DEADLIFT_ROM_MIN && rom   <= DEADLIFT_ROM_MAX &&
        dur   >= DEADLIFT_DUR_MIN && dur   <= DEADLIFT_DUR_MAX) {
        return ExerciseType::Deadlift;
    }

    // 2) SQUAT — slow cadence, moderate ROM, high peak accel on stand-up.
    if (pitchDom &&
        peakA >= SQUAT_PEAK_ACCEL_MIN &&
        rom   >= SQUAT_ROM_MIN && rom   <= SQUAT_ROM_MAX &&
        dur   >= SQUAT_DUR_MIN && dur   <= SQUAT_DUR_MAX) {
        return ExerciseType::Squat;
    }

    // 3) LATERAL RAISE — roll-dominant, very low pitch.
    if (rollDom &&
        f.rmsPitch < LATRAISE_PITCH_MAX &&
        rom   >= LATRAISE_ROM_MIN && rom   <= LATRAISE_ROM_MAX &&
        dur   >= LATRAISE_DUR_MIN && dur   <= LATRAISE_DUR_MAX) {
        return ExerciseType::LateralRaise;
    }

    // 4) ROW — pitch+roll combined motion (retraction signature).
    if ((pitchDom || rollDom) &&
        f.rmsPitch > 1.0f &&
        (f.rmsRoll / f.rmsPitch) >= ROW_COMBINED_RATIO_MIN &&
        rom   >= ROW_ROM_MIN && rom   <= ROW_ROM_MAX &&
        dur   >= ROW_DUR_MIN && dur   <= ROW_DUR_MAX) {
        return ExerciseType::Row;
    }

    // 5) OVERHEAD PRESS — large ROM + vertical accel. Differentiator vs
    //    shoulder press is the larger ROM (full lockout overhead).
    if (pitchDom &&
        peakA >= OHP_VACCEL_MIN &&
        rom   >= OHP_ROM_MIN && rom   <= OHP_ROM_MAX &&
        dur   >= OHP_DUR_MIN && dur   <= OHP_DUR_MAX) {
        return ExerciseType::OverheadPress;
    }

    // 6) SHOULDER PRESS — narrower ROM than OHP, still vertical accel.
    if (pitchDom &&
        peakA >= SHOULDER_VACCEL_MIN &&
        rom   >= SHOULDER_ROM_MIN && rom   <= SHOULDER_ROM_MAX &&
        dur   >= SHOULDER_DUR_MIN && dur   <= SHOULDER_DUR_MAX) {
        return ExerciseType::ShoulderPress;
    }

    // 7) BICEP CURL — pitch-dominant, large ROM, high angular velocity.
    if (pitchDom &&
        peakW >= BICEP_PEAK_OMEGA_MIN &&
        rom   >= BICEP_ROM_MIN && rom   <= BICEP_ROM_MAX &&
        dur   >= BICEP_DUR_MIN && dur   <= BICEP_DUR_MAX) {
        return ExerciseType::BicepCurl;
    }

    // 8) BENCH PRESS — moderate pitch ROM, low roll, slower cadence.
    if (pitchDom &&
        f.rmsRoll < BENCH_ROLL_MAX &&
        rom   >= BENCH_ROM_MIN && rom   <= BENCH_ROM_MAX &&
        dur   >= BENCH_DUR_MIN && dur   <= BENCH_DUR_MAX) {
        return ExerciseType::BenchPress;
    }

    // 9) TRICEP PUSHDOWN — quick cadence, pitch dominant, low roll.
    if (pitchDom &&
        f.rmsRoll < TRICEP_ROLL_MAX &&
        rom   >= TRICEP_ROM_MIN && rom   <= TRICEP_ROM_MAX &&
        dur   >= TRICEP_DUR_MIN && dur   <= TRICEP_DUR_MAX) {
        return ExerciseType::TricepPushdown;
    }

    return ExerciseType::Unknown;
}

// ---------------------------------------------------------------------------

void reset() {
    s_committed   = ExerciseType::Unknown;
    s_streakType  = ExerciseType::Unknown;
    s_streakCount = 0;
}

Result classify(const RepFeatures &f) {
    const ExerciseType t = classifyRaw(f);

    // Streak tracking — N consecutive identical classifications required.
    if (t != ExerciseType::Unknown && t == s_streakType) {
        if (s_streakCount < 255) ++s_streakCount;
    } else {
        s_streakType  = t;
        s_streakCount = (t == ExerciseType::Unknown) ? 0 : 1;
    }

    if (s_streakCount >= CLASSIFIER_CONFIRM_REPS) {
        s_committed = s_streakType;
    }

    Result r;
    r.type      = (s_committed != ExerciseType::Unknown) ? s_committed : t;
    r.confident = (s_committed != ExerciseType::Unknown);
    return r;
}

ExerciseType getCommitted() { return s_committed; }

const char *toString(ExerciseType t) {
    switch (t) {
        case ExerciseType::BicepCurl:      return "BICEP CURL";
        case ExerciseType::ShoulderPress:  return "SHOULDER PRESS";
        case ExerciseType::BenchPress:     return "BENCH PRESS";
        case ExerciseType::LateralRaise:   return "LATERAL RAISE";
        case ExerciseType::Squat:          return "SQUAT";
        case ExerciseType::Deadlift:       return "DEADLIFT";
        case ExerciseType::OverheadPress:  return "OVERHEAD PRESS";
        case ExerciseType::TricepPushdown: return "TRICEP PUSH";
        case ExerciseType::Row:            return "ROW";
        default:                           return "---";
    }
}

} // namespace exercise_classifier
