/**
 * @file    exercise_classifier.h
 * @brief   Heuristic classifier mapping per-rep features → exercise label.
 */

#ifndef REPSENSE_EXERCISE_CLASSIFIER_H
#define REPSENSE_EXERCISE_CLASSIFIER_H

#include <Arduino.h>
#include "rep_detector.h"

namespace exercise_classifier {

enum class ExerciseType : uint8_t {
    Unknown = 0,
    BicepCurl,
    ShoulderPress,
    BenchPress,
    LateralRaise,
    Squat,
    Deadlift,
    OverheadPress,
    TricepPushdown,
    Row,
};

struct Result {
    ExerciseType type;
    bool         confident;   ///< True once CLASSIFIER_CONFIRM_REPS in a row match.
};

/// @brief Reset committed exercise + streak counter.
void reset();

/// @brief Classify a single rep's features; updates internal streak / confidence.
Result classify(const rep_detector::RepFeatures &f);

/// @brief Last committed (confident) exercise type — UNKNOWN if none yet.
ExerciseType getCommitted();

/// @brief Human-readable name for the OLED.
const char *toString(ExerciseType t);

} // namespace exercise_classifier

#endif // REPSENSE_EXERCISE_CLASSIFIER_H
