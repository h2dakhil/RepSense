/**
 * @file    display.h
 * @brief   SSD1306 OLED UI — boot, active, rest, and sleep screens.
 */

#ifndef REPSENSE_DISPLAY_H
#define REPSENSE_DISPLAY_H

#include <Arduino.h>
#include "exercise_classifier.h"

namespace display {

/// @brief Bring up the SSD1306. Returns false if allocation/init fails.
bool begin();

/// @brief Project name + version for BOOT_SCREEN_MS, then clears.
void showBootScreen();

/// @brief Main active screen: exercise name, rep count, battery icon.
void showActive(exercise_classifier::ExerciseType type, int reps, int batteryPct);

/// @brief Between-sets screen with last set summary.
void showRest(exercise_classifier::ExerciseType lastType, int repsLastSet);

/// @brief Power the panel off entirely (DISPLAYOFF command).
void sleep();

/// @brief Re-enable the panel after sleep().
void wake();

} // namespace display

#endif // REPSENSE_DISPLAY_H
