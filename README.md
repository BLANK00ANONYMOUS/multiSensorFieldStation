# Multi-Sensor Field Station
### Arduino Uno + DHT11 + HC-SR04 + MPU-6050

A complete remote field monitoring station that acquires environmental, proximity, and motion data simultaneously from three sensors — with a live dashboard display, system-wide alert state machine, Serial command interface, and per-sensor statistics tracking.

Mirrors the architecture of real SCADA (Supervisory Control and Data Acquisition) field nodes deployed in oilfield operations, environmental monitoring stations, pipeline integrity systems, and offshore platform instrumentation.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│              MULTI-SENSOR FIELD STATION                  │
├───────────────┬──────────────────┬──────────────────────┤
│  DHT11        │  HC-SR04         │  MPU-6050            │
│  Temperature  │  Ultrasonic      │  Accelerometer       │
│  Humidity     │  Fill Level      │  Gyroscope           │
│  Heat Index   │  Distance        │  Roll / Pitch / Vib  │
├───────────────┴──────────────────┴──────────────────────┤
│              ALERT STATE MACHINE                         │
│         OK  →  WARNING  →  CRITICAL                     │
├─────────────────────────────────────────────────────────┤
│          SERIAL COMMAND INTERFACE                        │
│       s = dashboard   r = reset   a = alerts            │
└─────────────────────────────────────────────────────────┘
```

---

## Features

- Simultaneous 3-sensor data acquisition every 2 seconds
- System-wide alert state machine: OK / WARNING / CRITICAL
- Live dashboard view with all parameters, statistics, and level bar
- Graceful per-sensor failure handling — station continues if one sensor fails
- Rolling RMS vibration measurement over 10-sample history
- Automatic MPU-6050 calibration on startup
- Median filtering on HC-SR04 readings for noise elimination
- Session statistics: max/min for all channels, total readings, total alerts
- Serial command interface for runtime control
- Uptime tracking

---

## Sensors

| Sensor | Measures | Protocol |
|---|---|---|
| DHT11 | Temperature (°C), Humidity (%), Heat Index | Single-wire |
| HC-SR04 | Distance (cm) → Fill Level (%) | Digital pulse timing |
| MPU-6050 | Roll (°), Pitch (°), Vibration (g) | I2C |

---

## Hardware Required

| Component | Quantity |
|---|---|
| Arduino Uno | 1 |
| DHT11 temperature/humidity sensor | 1 |
| HC-SR04 ultrasonic distance sensor | 1 |
| MPU-6050 IMU (GY-521 breakout) | 1 |
| Jumper wires | ~11 |

---

## Wiring

```
DHT11           →    Arduino Uno
────────────────────────────────
VCC             →    5V
GND             →    GND
DATA            →    Pin 2

HC-SR04         →    Arduino Uno
────────────────────────────────
VCC             →    5V
GND             →    GND
TRIG            →    Pin 9
ECHO            →    Pin 10

MPU-6050        →    Arduino Uno
────────────────────────────────
VCC             →    3.3V  ← NOT 5V
GND             →    GND
SDA             →    A4
SCL             →    A5
```

---

## Libraries Required

Install via Arduino IDE → Sketch → Include Library → Manage Libraries:

- **DHT sensor library** by Adafruit
- **Adafruit Unified Sensor** by Adafruit
- **MPU6050** by Electronic Cats

---

## Serial Commands

Type in Arduino IDE Serial Monitor (baud: 9600):

| Command | Action |
|---|---|
| `s` | Print full status dashboard immediately |
| `r` | Reset all session statistics |
| `a` | Toggle alert messages on/off |
| `h` | Show help menu |

---

## Serial Output

```
  ╔══════════════════════════════════════════════════╗
  ║        MULTI-SENSOR FIELD STATION  v1.0          ║
  ║   DHT11  |  HC-SR04  |  MPU-6050                 ║
  ╚══════════════════════════════════════════════════╝

[00:00:02] T:28.0C H:65.0% Lv:52.3% Ro:0.4° Vi:0.012g [OK      ]  #1
[00:00:04] T:28.0C H:65.0% Lv:52.1% Ro:18.2° Vi:0.045g [WARNING ]  #2
  [!] Tilt Warning: 18.2deg

  ╔════════════════════════════════════════════════╗
  ║         FIELD STATION DASHBOARD                ║
  ╠════════════════════════════════════════════════╣
  ║  Uptime  : 00:00:10     Readings: 5            ║
  ║  System  : [   OK      ]   Alerts: 1           ║
  ╠════════════════════════════════════════════════╣
  ║  [ENVIRONMENTAL — DHT11]                       ║
  ║    Temperature : 28.0 C  (max: 29.1 / min: 27.5 C) ║
  ║    Humidity    : 65.0 %  (max: 66.0 / min: 64.0 %) ║
  ║    Heat Index  : 30.2 C                        ║
  ╠════════════════════════════════════════════════╣
  ║  [LEVEL MONITOR — HC-SR04]                     ║
  ║    Distance    : 14.2 cm                       ║
  ║    Fill Level  : 52.3 %  [████████░░░░░░░]     ║
  ║    Range       : max 55.1% / min 49.8%         ║
  ╠════════════════════════════════════════════════╣
  ║  [MOTION & TILT — MPU-6050]                    ║
  ║    Roll        : 0.4 deg                       ║
  ║    Pitch       : 0.8 deg                       ║
  ║    Vibration   : 0.012 g  (peak: 0.231 g)      ║
  ║    Max Tilt    : 18.2 deg                      ║
  ╚════════════════════════════════════════════════╝
```

---

## Alert Thresholds

### Environmental (DHT11)
| Parameter | Warning |
|---|---|
| Temperature | > 40°C or < 5°C |
| Humidity | > 80% or < 20% |

### Level (HC-SR04)
| Parameter | Warning | Critical |
|---|---|---|
| Fill Level | < 20% or > 90% | < 10% |

### Motion (MPU-6050)
| Parameter | Warning | Critical |
|---|---|---|
| Tilt | > 15° | > 30° |
| Vibration | > 0.3g | > 0.8g |

---

## Real-World Relevance

This project demonstrates a complete field instrumentation stack:

- **Multi-sensor fusion** — simultaneous acquisition from heterogeneous sensors
- **Mixed protocols** — Single-wire (DHT11), digital pulse (HC-SR04), I2C (MPU-6050)
- **State machine design** — system-level alert states mirror SCADA alarm management
- **Signal processing** — median filtering and RMS calculation on sensor streams
- **Sensor calibration** — automatic IMU zero-offset compensation
- **Fault tolerance** — station continues operating if individual sensors fail
- **Command interface** — runtime control mirrors field device configuration
- **Statistics tracking** — data quality monitoring fundamental to field operations

---

## Project Structure

```
MultiSensorFieldStation/
├── MultiSensorFieldStation.ino    # Main Arduino sketch (~400 lines)
└── README.md                      # This file
```

---

## Portfolio Context

This is the capstone project of a 5-part embedded systems portfolio:

| # | Project | Skills Demonstrated |
|---|---|---|
| 1 | Environmental Data Logger | DHT11, single-wire, threshold alerting |
| 2 | Ultrasonic Level Monitor | HC-SR04, pulse timing, median filtering |
| 3 | Vibration & Tilt Monitor | MPU-6050, I2C, RMS, calibration |
| 4 | *(Gas Detection)* | ADC, analog sensors, safety systems |
| **5** | **Multi-Sensor Field Station** | **Full system integration, state machine, command interface** |

---

## Author
Utkarsh — Embedded Systems & Field Engineering Portfolio  
Targeting Field Engineer roles at SLB, Halliburton, Baker Hughes, and Fugro.  
Building hands-on expertise in sensor interfacing, embedded C, and field instrumentation systems.
