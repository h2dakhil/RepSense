# Wrist Fitness Tracker — Custom PCB

A wrist-mounted, watch-form wearable that detects and counts gym exercise reps in real time using an IMU and sensor fusion. Built around a circular 42 mm custom PCB designed for low power, low cost, and size-constrained wearable use.

Inspired by the Apple Watch workout detection feature but built from scratch — custom hardware, custom firmware, no cloud dependency.

---

## Overview

The device sits on the wrist like a watch. It reads 6-axis motion data from an IMU, runs an Extended Kalman Filter (EKF) to fuse accelerometer and gyroscope data into a clean orientation estimate, and uses that to detect and count repetitions for exercises including:

- Bicep curl
- Shoulder press
- Bench press
- Lateral raise
- Squat
- Deadlift
- Overhead press

Rep count and exercise type are displayed in real time on a small OLED screen. The entire signal processing pipeline runs on-device with no wireless transmission required in v1.

---

## Hardware

### System block diagram

```
USB-C ──────────────────────────────────┐
                                        ▼
LiPo battery ──────────────► BQ24074 PMIC ──► VDDH
                              (power path +         │
                               LiPo charger)        ▼
                                             nRF52840 QFN73
                                             internal DC-DC
                                             REG0 (DCDCEN0)
                                                    │
                                             3.3V EXTSUPPLY
                                            ┌───────┴────────┐
                                            ▼                ▼
                                       LSM6DSV IMU       OLED display
                                       (I2C 0x6A)        (I2C 0x78)
```

### MCU — nRF52840 QIAA QFN73

Nordic's nRF52840 was selected because it satisfies every constraint in a single chip with no external LDO required:

- Cortex-M4F at 64 MHz with hardware floating point — needed for EKF math
- Native USB (D+/D−) — drag-and-drop UF2 firmware flashing, no UART bridge chip
- BLE 5.0 built in — phone integration ready for v2 with no extra hardware
- Internal DC-DC converter (DCDCEN0) — steps VDDH down to 3.3V internally, powers entire board via EXTSUPPLY rail
- 1.5 µA deep sleep with wake-on-interrupt — critical for wearable battery life
- VDDH accepts up to 5.5V — direct PMIC output connection, no external LDO needed
- Same chip as the Seeed XIAO nRF52840 Sense used for firmware feasibility validation

**Circuit configuration:** Nordic Config 4 (High Voltage mode, DCDCEN0 enabled, USB enabled, NFC disabled). The PMIC SYS output feeds VDDH directly. The chip's internal REG0 DC-DC produces a stable 3.3V on VDD which is exposed as the board's entire 3.3V supply rail via EXTSUPPLY. One 10 µH inductor on DCCH is the only passive required for this configuration.

**Programming:** First-time bootloader flash via SWD using a J-Link EDU Mini and Tag-Connect TC2030-IDC-NL pads on the PCB. After the Adafruit UF2 bootloader is installed, all subsequent firmware updates are drag-and-drop `.uf2` over USB — identical to the Seeed dev board experience.

**Package selection:** QFN48 was considered and rejected. The QFN48 does not expose D+, D−, or VBUS — USB is physically absent. QFN73 is required. The 1 mm size difference per side is irrelevant at 42 mm PCB diameter.

### IMU — LSM6DSVTR

ST's LSM6DSV was selected for its onboard sensor fusion engine and FSM (Finite State Machine) which offload motion processing from the MCU:

- 6-axis: 3-axis accelerometer (±2 to ±16 g) + 3-axis gyroscope (up to 4000 dps)
- Onboard sensor fusion reduces MCU interrupt load
- Programmable FSM enables gesture and motion pattern detection in hardware
- Large FIFO for buffered data collection, reducing MCU wake frequency
- Low power always-on mode for wrist-raise and motion-triggered wake
- I2C interface, address 0x6A (SA0 pulled to GND)
- INT1 routed to P1.06 — configured as wake-on-motion interrupt

INT1 is the key to efficient battery use. The nRF52840 sleeps at 1.5 µA. When the IMU detects motion above a threshold it pulls INT1 high, waking the MCU instantly. The MCU only runs the rep counting pipeline while the user is actively moving. Between sets and between sessions the system is effectively off.

**I2C pin assignment:** SDA → P1.04, SCL → P1.05, INT1 → P1.06

### PMIC — BQ24074RGTR

TI's BQ24074 handles charging and system power path in a single 3×3 mm QFN:

- Dynamic Power Path Management (DPPM) — simultaneously powers system and charges battery from USB, reducing battery charge/discharge cycles
- USB500 mode selected (EN1 high, EN2 low) — 500 mA input limit, leaves ~400 mA headroom for system after 100 mA charge current
- Charge current set to 100 mA (0.5C for 200 mAh cell) via 9 kΩ ISET resistor
- Input range 4.35–10.5V — compatible with USB 5V
- OVP, thermal regulation, reverse current, and short-circuit protection built in
- TS pin tied to 10 kΩ to GND — battery has no NTC thermistor, this prevents false thermal fault

### Display

0.91" SSD1306 OLED, 128×32 monochrome, I2C address 0x78. Shares I2C bus with IMU — no address conflict. Max 30 mA. Soldered directly to 4× 2.54 mm PTH pads on PCB. Onboard module pullup resistors (4.7 kΩ) serve the entire I2C bus.

### Battery

302035 LiPo, 3.7V nominal, 200 mAh, JST-PH 2 mm 2-pin. Dimensions 36×20×3 mm. Includes onboard protection PCB (OVP, overdischarge, short circuit). Charge current 100 mA (0.5C).

Battery voltage monitored via 1 MΩ/1 MΩ resistor divider to nRF52840 AIN pin for on-screen battery percentage.

---

## Power budget

Design target: low enough average current draw to sustain multi-hour gym sessions on a 200 mAh cell.

### Active current draw (during rep counting)

| Component | Current | Notes |
|---|---|---|
| nRF52840 (active, no BLE) | ~5 mA | Cortex-M4 running EKF + display update |
| LSM6DSV IMU | ~0.35 mA | 3.3V estimated from 0.65 mA at 1.8V |
| OLED display | ~20 mA | Typical for 128×32 at moderate brightness |
| PMIC quiescent | ~0.05 mA | BQ24074 Iq |
| Resistor divider | ~0.002 mA | 2.1 µA, negligible |
| **Total active** | **~25.4 mA** | |

### Sleep current (between sets / idle)

| Component | Current | Notes |
|---|---|---|
| nRF52840 (System ON, wake on RTC) | 1.5 µA | Nordic spec, no RAM retention |
| LSM6DSV (low power accel only) | ~25 µA | Motion detection active, waiting for INT1 |
| OLED display (off) | ~5 µA | Controller standby |
| **Total sleep** | **~32 µA** | |

### Runtime estimate

Assuming a typical 1-hour gym session: 20 minutes active (rep counting) + 40 minutes rest (sleep between sets):

```
Active:  20 min × 25.4 mA  = 508 mAh-min
Sleep:   40 min × 0.032 mA = 1.28 mAh-min
Total:   509 mAh-min / 60  = ~8.5 mAh per session
```

On a 200 mAh cell (usable ~160 mAh accounting for LDO dropout margin):

```
160 mAh / 8.5 mAh per session ≈ ~18 full gym sessions per charge
```

For all-day wear with occasional active periods, sleep current dominates — 32 µA into 200 mAh gives over 250 days of idle standby. In practice charge weekly.

### Battery monitoring

Resistor divider (1 MΩ / 1 MΩ) scales VBAT to half voltage at the ADC pin:

| Battery state | VBAT | AIN voltage | 12-bit ADC count |
|---|---|---|---|
| Full | 4.20 V | 2.10 V | 2606 |
| 75% | 3.90 V | 1.95 V | 2420 |
| 50% | 3.70 V | 1.85 V | 2296 |
| 25% | 3.50 V | 1.75 V | 2172 |
| Low | 3.20 V | 1.60 V | 1986 |

Firmware reverse calculation: `VBAT = ADC_count × 6.6 / 4096`

Divider quiescent drain: 2.1 µA — equivalent to 5.4 years of drain on this cell. Negligible.

---

## Firmware architecture

### Sensing pipeline

```
IMU (6-axis raw) → EKF sensor fusion → orientation estimate
                                              │
                                    rep detection algorithm
                                              │
                                    exercise classifier (heuristic v1)
                                              │
                                       display update
```

### Extended Kalman Filter (EKF)

Raw accelerometer and gyroscope data are fused using an EKF:

- **Prediction step:** gyroscope data integrates orientation forward in time — fast but drifts
- **Correction step:** accelerometer gravity vector corrects the drift — slow but absolute
- **Output:** clean, drift-free quaternion orientation estimate

On a wrist-mounted IMU, gravity rotates in the body frame throughout each rep. This rotation is both the challenge (it confounds raw accelerometer rep detection) and the signal (it gives the EKF its correction input). The EKF output removes gravity ambiguity and produces a clean tilt signal that reps appear as clear periodic waves in.

The nRF52840 Cortex-M4F with hardware floating point handles EKF math at >500 Hz comfortably.

### Rep detection

Per rep, the following features are extracted from the EKF orientation output:

- Dominant motion axis (principal direction of angular change)
- Range of motion in degrees
- Rep duration
- Peak angular velocity
- Zero-crossing frequency

**v1 — Heuristic classifier:** if/else rules on extracted features. Fast, interpretable, no training data required. Handles 5–8 distinct exercises reliably.

**v2 — TinyML:** TensorFlow Lite Micro CNN/MLP on rep window. Better generalisation across users and subtle exercise variations. Planned once v1 heuristics are validated.

### Exercise detection logic

Each exercise has a distinct wrist motion signature:

| Exercise | Primary axis | Motion signature |
|---|---|---|
| Bicep curl | Pitch (wrist rotates forward) | Large pitch swing, ~120° ROM, 1–2 s period |
| Shoulder press | Vertical translation dominant | High Z accel, moderate pitch, pressing pattern |
| Bench press | Elbow extension dominant | Moderate pitch, low ROM, bilateral pattern |
| Lateral raise | Roll (arm lifts sideways) | Large roll swing, ~90° ROM |
| Squat | Body pitch with knee flexion | Slow deep pitch oscillation, ~3–4 s |
| Deadlift | Hip hinge dominant | Large pitch, high peak accel on concentric |
| Overhead press | Similar to shoulder press | Higher ROM, full elbow extension signature |

The classifier reads the dominant axis, ROM range, and rep cadence. Most exercises are distinct enough that a threshold-based decision tree separates them reliably. Exercises with similar signatures (shoulder press vs overhead press) are disambiguated by ROM magnitude.

### Wake-on-motion

IMU INT1 configured as motion-detect interrupt → nRF52840 GPIO wakeup. MCU stays in System ON sleep (1.5 µA) until motion above threshold detected. Eliminates polling entirely.

---

## PCB

- **Form factor:** circular, 42 mm diameter
- **Layers:** 2
- **Manufacturer:** JLCPCB (fabrication + assembly)
- **Assembly:** JLCPCB SMT — nRF52840 QFN73, BQ24074 QFN16, LSM6DSV LGA14 all assembled
- **Programming interface:** Tag-Connect TC2030-IDC-NL pads (no legs, flush with PCB surface)
- **Reset:** SMD momentary button, RESET pin → GND, 10 kΩ pullup, 100 nF filter cap
- **Antenna:** unpopulated in v1 (BLE not enabled). RF matching network footprints retained DNP for v2 population without board respin
- **Crystal:** 32 MHz SMD2016 (YXC X2016 32MKB4SI, LCSC C718072) mandatory main clock. 32.768 kHz RTC crystal footprint DNP for v2

---

## Estimated unit cost

Based on JLCPCB pricing at low volume (5 units):

| Item | Est. cost (USD) |
|---|---|
| PCB fabrication (5× circular 42 mm, 2-layer) | ~$2.00/unit |
| nRF52840 QIAA QFN73 (LCSC C190794) | ~$6.50 |
| LSM6DSVTR IMU | ~$3.50 |
| BQ24074RGTR PMIC | ~$2.50 |
| 32 MHz crystal | ~$0.10 |
| 10 µH inductor (DCCH) | ~$0.15 |
| Passives (caps, resistors, ESD) | ~$0.50 |
| USB-C connector | ~$0.30 |
| JST-PH battery connector | ~$0.20 |
| OLED display module | ~$3.00 |
| LiPo 302035 200 mAh | ~$3.00 |
| JLCPCB SMT assembly fee (est.) | ~$8.00 |
| **Total per unit (5-unit run)** | **~$29.75** |

Cost drops significantly at higher volumes — MCU and PMIC pricing improves at 50+ units and assembly amortises across more boards.

---

## Feasibility validation

Before committing to the custom PCB, the full sensing pipeline was prototyped and validated on a **Seeed XIAO nRF52840 Sense** development board. This board uses the same nRF52840 MCU and includes an onboard LSM6DS3 IMU (predecessor to the LSM6DSV used in the final design).

Validation confirmed:
- IMU data acquisition over I2C at the required sample rate
- Raw accelerometer and gyroscope signal quality for rep detection
- Wrist-mounted motion signatures are clearly periodic and distinguishable per exercise
- nRF52840 toolchain (Adafruit nRF52 Arduino framework) workflow — compile, UF2 drag-and-drop flash, serial monitor
- EKF algorithm correctness on real motion data before hardware commitment

The custom PCB is a direct hardware consolidation of the validated dev board pipeline — same MCU, same firmware framework, same IMU family — into a circular wearable form factor.

---

## Repository structure

```
/firmware          nRF52840 Arduino/PlatformIO firmware
/hardware          KiCad schematic and PCB layout files
/docs              design notes, component datasheets, calculations
/data              CSV motion logs from feasibility phase
README.md
```

---

## Roadmap

| Version | Status | Description |
|---|---|---|
| v1 | In progress | Custom PCB, heuristic rep counter, OLED display, 5 exercises |
| v2 | Planned | BLE to iPhone, TinyML classifier, 10+ exercises, OTA firmware updates |
| v3 | Future | Custom enclosure, production-ready assembly, refined form factor |

---

## Design constraints summary

| Constraint | Approach |
|---|---|
| Size | 42 mm circular PCB, no external LDO, all ICs QFN/LGA |
| Low power | nRF52840 1.5 µA sleep, IMU wake-on-motion INT1, display off between updates |
| Low cost | ~$30/unit at 5 units, ~$18 at volume; single-sided JLCPCB assembly |
| Programmability | Native USB UF2 after one-time SWD bootloader flash |
| Sensor accuracy | EKF fusion eliminates gyro drift and gravity confusion on raw accel |
