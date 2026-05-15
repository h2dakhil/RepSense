/**
 * @file    battery.h
 * @brief   LiPo battery voltage sense + non-linear percentage mapping.
 */

#ifndef REPSENSE_BATTERY_H
#define REPSENSE_BATTERY_H

#include <Arduino.h>

namespace battery {

/// @brief Configure the SAADC: 12-bit, VDD/4 reference, 1/6 gain, AIN input.
void begin();

/// @brief Sample if at least BATTERY_SAMPLE_MS has elapsed since last read.
void update();

float   getBatteryVoltage();   ///< Volts (cached)
uint8_t getBatteryPercent();   ///< 0–100 (cached)

} // namespace battery

#endif // REPSENSE_BATTERY_H
