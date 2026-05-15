/**
 * @file    power.h
 * @brief   Power management: DC-DC enable, low-power peripheral shutdown,
 *          System ON sleep with GPIO wake on IMU INT1.
 */

#ifndef REPSENSE_POWER_H
#define REPSENSE_POWER_H

#include <Arduino.h>

namespace power {

/// @brief Enable the nRF52840 internal DC-DC converter (REG1 / DCDCEN).
void enableDCDC();

/// @brief Disable unused peripherals (UART, SPI, etc.) to minimise leakage.
void configureLowPowerPeripherals();

/// @brief Re-enable peripherals needed for the ACTIVE state after wake.
void restoreActivePeripherals();

/// @brief Enter System ON sleep. Returns when GPIO wake (INT1) fires.
void enterSleep();

/// @brief ISR hook called from the INT1 attachInterrupt callback.
void onWakeInterrupt();

/// @brief True if a wake interrupt has been latched since last clear.
bool wakeRequested();

/// @brief Clear the latched wake flag.
void clearWake();

} // namespace power

#endif // REPSENSE_POWER_H
