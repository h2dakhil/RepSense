/**
 * @file    battery.cpp
 * @brief   Battery sense via nRF52 SAADC.
 *
 * Hardware path: VBAT → 1 MΩ / 1 MΩ divider → AIN. The Adafruit Feather
 * routes this to A6 (P0.05 / AIN3). With the SAADC set to VDD reference
 * (3.3 V), 12-bit resolution, and 1/2 gain on the divided input:
 *
 *     VBAT = ADC_count · (2 · 3.3 V) / 4096 = ADC_count · 6.6 / 4096.
 *
 * The LiPo discharge curve is highly non-linear — a linear voltage→%
 * mapping over-reports the middle of the curve. We piecewise-interpolate
 * an 8-point lookup table that matches a typical 3.7 V LiPo cell.
 */

#include "battery.h"
#include "config.h"

namespace battery {

// Discharge curve: (voltage, percent), descending by voltage.
struct CurvePoint { float v; uint8_t pct; };
static const CurvePoint kCurve[] = {
    { 4.20f, 100 },
    { 4.00f,  85 },
    { 3.85f,  70 },
    { 3.75f,  55 },
    { 3.65f,  40 },
    { 3.50f,  20 },
    { 3.30f,   5 },
    { 3.00f,   0 },
};
static constexpr uint8_t kCurveN = sizeof(kCurve) / sizeof(kCurve[0]);

static float    s_voltage    = 0.0f;
static uint8_t  s_percent    = 0;
static uint32_t s_lastSample = 0;

// ---------------------------------------------------------------------------

void begin() {
    // analogReference / analogReadResolution are provided by the Adafruit
    // nRF52 core. AR_INTERNAL_3_0 is 3.0 V — VDD reference is the right
    // choice here, exposed as AR_VDD4 (VDD/4 ref + 1/4 gain = VDD full-scale).
    analogReference(AR_VDD4);
    analogReadResolution(12);
    pinMode(BATTERY_AIN, INPUT);
    s_lastSample = 0;   // force first read on next update()
    update();
}

// ---------------------------------------------------------------------------

static uint8_t voltageToPercent(float v) {
    if (v >= kCurve[0].v) return kCurve[0].pct;
    if (v <= kCurve[kCurveN - 1].v) return kCurve[kCurveN - 1].pct;
    for (uint8_t i = 0; i < kCurveN - 1; ++i) {
        const float vHi = kCurve[i].v,     vLo = kCurve[i + 1].v;
        const uint8_t pHi = kCurve[i].pct, pLo = kCurve[i + 1].pct;
        if (v <= vHi && v >= vLo) {
            const float t = (v - vLo) / (vHi - vLo);  // 0 at low, 1 at high
            return (uint8_t)(pLo + t * (pHi - pLo));
        }
    }
    return 0;
}

// ---------------------------------------------------------------------------

void update() {
    const uint32_t now = millis();
    if (s_lastSample != 0 && (now - s_lastSample) < BATTERY_SAMPLE_MS) return;
    s_lastSample = now;

    const uint16_t raw = analogRead(BATTERY_AIN);
    s_voltage = raw * BATTERY_ADC_TO_VOLTS;
    s_percent = voltageToPercent(s_voltage);

#ifdef DEBUG
    Serial.print("[battery] raw=");  Serial.print(raw);
    Serial.print(" V=");             Serial.print(s_voltage, 3);
    Serial.print(" pct=");           Serial.println(s_percent);
#endif
}

float   getBatteryVoltage() { return s_voltage; }
uint8_t getBatteryPercent() { return s_percent; }

} // namespace battery
