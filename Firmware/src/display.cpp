/**
 * @file    display.cpp
 * @brief   SSD1306 OLED renderer.
 *
 * Layout (128 × 32):
 *   row 0 (y= 0..7):  exercise name (Adafruit GFX size 1) + battery icon at right
 *   row 1 (y= 8..31): rep count (size 3) centred-left
 *
 * Partial redraws: we cache last-rendered values and only repaint the
 * affected region — full-frame redraws cause visible flicker on the SSD1306.
 */

#include "display.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

namespace display {

static Adafruit_SSD1306 oled(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, -1);

// Last-drawn cache for partial redraws.
static exercise_classifier::ExerciseType s_lastType = exercise_classifier::ExerciseType::Unknown;
static int  s_lastReps    = -1;
static int  s_lastBattery = -1;
static bool s_inActive    = false;

// ---------------------------------------------------------------------------

bool begin() {
    if (!oled.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDR_7BIT)) return false;
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.display();
    return true;
}

// ---------------------------------------------------------------------------

void showBootScreen() {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(FIRMWARE_NAME);
    oled.setCursor(0, 12);
    oled.print("v"); oled.print(FIRMWARE_VERSION);
    oled.setCursor(0, 24);
    oled.print("ready");
    oled.display();
    delay(BOOT_SCREEN_MS);
    oled.clearDisplay();
    oled.display();
}

// ---------------------------------------------------------------------------
// Battery icon: a 14×7 rounded outline with proportional fill.
// Drawn into the top-right corner from x=110..123.
// ---------------------------------------------------------------------------
static void drawBatteryIcon(int pct) {
    const int16_t x = 110, y = 0;
    oled.fillRect(x, y, 18, 8, SSD1306_BLACK);   // clear region
    oled.drawRect(x, y, 14, 7, SSD1306_WHITE);   // body
    oled.drawLine(x + 14, y + 2, x + 14, y + 4, SSD1306_WHITE); // nub
    const int fillW = (pct * 12) / 100;
    if (fillW > 0) oled.fillRect(x + 1, y + 1, fillW, 5, SSD1306_WHITE);
}

// ---------------------------------------------------------------------------

void showActive(exercise_classifier::ExerciseType type, int reps, int batteryPct) {
    // First entry to active screen forces a full redraw.
    if (!s_inActive) {
        oled.clearDisplay();
        oled.display();
        s_lastType    = (exercise_classifier::ExerciseType)0xFF;
        s_lastReps    = -1;
        s_lastBattery = -1;
        s_inActive    = true;
    }

    bool dirty = false;

    // Exercise name (top-left, size 1).
    if (type != s_lastType) {
        oled.fillRect(0, 0, 108, 8, SSD1306_BLACK);
        oled.setTextSize(1);
        oled.setCursor(0, 0);
        oled.print(exercise_classifier::toString(type));
        s_lastType = type;
        dirty = true;
    }

    // Rep count (centred-left, size 3 → 18 px per glyph).
    if (reps != s_lastReps) {
        oled.fillRect(0, 10, 110, 22, SSD1306_BLACK);
        oled.setTextSize(3);
        oled.setCursor(2, 10);
        oled.print(reps);
        s_lastReps = reps;
        dirty = true;
    }

    // Battery icon (top-right).
    if (batteryPct != s_lastBattery) {
        drawBatteryIcon(batteryPct);
        s_lastBattery = batteryPct;
        dirty = true;
    }

    if (dirty) oled.display();
}

// ---------------------------------------------------------------------------

void showRest(exercise_classifier::ExerciseType lastType, int repsLastSet) {
    s_inActive = false;
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setCursor(36, 0);
    oled.print("REST");
    oled.setTextSize(1);
    oled.setCursor(0, 18);
    oled.print(exercise_classifier::toString(lastType));
    oled.setCursor(0, 26);
    oled.print(repsLastSet); oled.print(" reps");
    oled.display();
}

// ---------------------------------------------------------------------------

void sleep() {
    oled.ssd1306_command(SSD1306_DISPLAYOFF);
    s_inActive = false;
}

void wake() {
    oled.ssd1306_command(SSD1306_DISPLAYON);
}

} // namespace display
