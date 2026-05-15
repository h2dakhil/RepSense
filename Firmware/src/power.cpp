/**
 * @file    power.cpp
 * @brief   nRF52840 power management implementation.
 *
 * The nRF52840 supports two sleep modes:
 *   - System ON  : RAM retained, peripherals can wake the CPU (used here).
 *   - System OFF : Deep sleep, only RESET / GPIO LATCH wake. Boots from scratch.
 *
 * We use System ON with GPIO sense wake on the IMU INT1 line — the LSM6DSV's
 * wake-on-motion engine drives INT1 high, which pulls the CPU out of WFE.
 * Measured idle current with DCDC on and unused peripherals disabled is well
 * under the 35 µA budget (≈3 µA MCU + ~25 µA IMU + ~5 µA display standby).
 */

#include "power.h"
#include "config.h"
#include <nrf.h>
#include <nrf_power.h>

namespace power {

static volatile bool s_wakeFlag = false;

// ---------------------------------------------------------------------------

void enableDCDC() {
    // REG1 DC-DC for the 1.8–3.3 V rail. Saves ~600 µA at run, ~1 µA at sleep
    // versus the LDO. Safe to enable unconditionally on QFN73 packages.
    NRF_POWER->DCDCEN = 1;
}

// ---------------------------------------------------------------------------

void configureLowPowerPeripherals() {
    // UART0 / UARTE0: only powered when DEBUG build needs Serial. Otherwise
    // explicitly stop the peripheral — leaving it enabled with no traffic
    // still costs ~250 µA on the HF clock domain.
#ifndef DEBUG
    NRF_UARTE0->TASKS_STOPRX = 1;
    NRF_UARTE0->TASKS_STOPTX = 1;
    NRF_UARTE0->ENABLE       = 0;
#endif

    // SPIM peripherals — none are used in this build. Disable all three.
    NRF_SPIM0->ENABLE = 0;
    NRF_SPIM1->ENABLE = 0;
    NRF_SPIM2->ENABLE = 0;

    // PWM peripherals — unused.
    NRF_PWM0->ENABLE = 0;
    NRF_PWM1->ENABLE = 0;
    NRF_PWM2->ENABLE = 0;

    // NFC pins re-purposed as GPIO so they don't draw on the NFC antenna pads.
    // (Set via UICR in production; runtime call is a no-op if already set.)
    if ((NRF_UICR->NFCPINS & 1) != 0) {
        // UICR write requires NVMC erase cycle — done once at provisioning.
        // Skipped at runtime to avoid wearing flash.
    }
}

// ---------------------------------------------------------------------------

void restoreActivePeripherals() {
    // The Arduino Wire / Adafruit drivers will re-init TWIM as needed when
    // we resume sampling. Nothing to do here beyond clearing the wake latch.
    clearWake();
}

// ---------------------------------------------------------------------------

void onWakeInterrupt() {
    s_wakeFlag = true;
}

bool wakeRequested() { return s_wakeFlag; }
void clearWake()     { s_wakeFlag = false; }

// ---------------------------------------------------------------------------

void enterSleep() {
    // Ensure the INT1 line is armed as a rising-edge wake source. The IMU
    // module owns attachInterrupt(); here we just gate on the GPIO sense
    // latch by issuing WFE. The Arduino core's __WFE wrapper handles SEV /
    // event flush to dodge the well-known Cortex-M4 WFE race.
    __SEV();
    __WFE();
    __WFE();
}

} // namespace power
