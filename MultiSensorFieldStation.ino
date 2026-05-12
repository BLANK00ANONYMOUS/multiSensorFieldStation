/*
 * ============================================================
 *  MULTI-SENSOR FIELD STATION
 *  Author  : Utkarsh
 *  Hardware: Arduino Uno + DHT11 + HC-SR04 + MPU-6050
 *  Version : 1.0.0
 * ============================================================
 *
 *  DESCRIPTION:
 *  A complete remote field monitoring station that reads
 *  environmental, proximity, and motion data simultaneously
 *  from three sensors, maintains a live dashboard display,
 *  manages system-wide alert states, and accepts Serial
 *  commands for runtime configuration.
 *
 *  Mirrors the architecture of real SCADA (Supervisory Control
 *  and Data Acquisition) field nodes deployed in oilfield
 *  operations, environmental monitoring stations, pipeline
 *  integrity systems, and offshore platform instrumentation.
 *
 *  SENSORS:
 *  [1] DHT11     — Temperature and Humidity (environmental)
 *  [2] HC-SR04   — Ultrasonic distance → fill level (proximity)
 *  [3] MPU-6050  — Accelerometer + Gyroscope (motion/tilt)
 *
 *  FEATURES:
 *  - Simultaneous multi-sensor data acquisition
 *  - System-wide alert state machine (OK/WARNING/CRITICAL)
 *  - Live dashboard with all parameters in one view
 *  - Serial command interface for runtime control
 *  - Per-sensor and system-level statistics
 *  - Uptime tracking and reading counter
 *  - Graceful sensor failure handling per channel
 *
 *  SERIAL COMMANDS (type in Serial Monitor):
 *  's' — Print full status dashboard
 *  'r' — Reset session statistics
 *  'a' — Toggle alert messages on/off
 *  'h' — Print help menu
 *
 *  WIRING:
 *  DHT11    VCC  -> 5V      GND -> GND    DATA -> Pin 2
 *  HC-SR04  VCC  -> 5V      GND -> GND    TRIG -> Pin 9    ECHO -> Pin 10
 *  MPU-6050 VCC  -> 3.3V    GND -> GND    SDA  -> A4       SCL  -> A5
 * ============================================================
 */

#include <DHT.h>
#include <Wire.h>
#include <MPU6050.h>

// ── Pin Configuration ────────────────────────────────────────
#define DHT_PIN          2
#define DHT_TYPE         DHT11
#define TRIG_PIN         9
#define ECHO_PIN         10
// MPU-6050 uses I2C: SDA=A4, SCL=A5 (hardware fixed)

// ── Tank Configuration ───────────────────────────────────────
#define TANK_HEIGHT_CM   30.0
#define SENSOR_OFFSET_CM  2.0

// ── Environmental Thresholds ─────────────────────────────────
#define TEMP_HIGH        40.0    // °C
#define TEMP_LOW          5.0    // °C
#define HUMIDITY_HIGH    80.0    // %
#define HUMIDITY_LOW     20.0    // %

// ── Level Thresholds ─────────────────────────────────────────
#define LEVEL_LOW        20.0    // %
#define LEVEL_CRITICAL   10.0    // %
#define LEVEL_HIGH       90.0    // %

// ── Motion Thresholds ────────────────────────────────────────
#define TILT_WARNING     15.0    // degrees
#define TILT_CRITICAL    30.0    // degrees
#define VIB_WARNING       0.3    // g
#define VIB_CRITICAL      0.8    // g

// ── Timing ───────────────────────────────────────────────────
#define SAMPLE_INTERVAL_MS   2000   // Full read cycle every 2s
#define DASHBOARD_INTERVAL      5   // Dashboard every N readings
#define CALIB_SAMPLES         150   // MPU calibration samples
#define VIB_HISTORY            10   // RMS window size

// ── Sensor Scale Factors ─────────────────────────────────────
#define ACCEL_SCALE      16384.0
#define GYRO_SCALE         131.0

// ── System Alert States ──────────────────────────────────────
typedef enum {
  STATE_OK       = 0,
  STATE_WARNING  = 1,
  STATE_CRITICAL = 2
} SystemState;

// ── Sensor Data Structures ───────────────────────────────────
struct EnvData {
  float temperature;
  float humidity;
  float heatIndex;
  bool  valid;
};

struct LevelData {
  float distanceCm;
  float levelPct;
  bool  valid;
};

struct MotionData {
  float roll;
  float pitch;
  float vibration;
  bool  valid;
};

// ── Global Objects ───────────────────────────────────────────
DHT     dht(DHT_PIN, DHT_TYPE);
MPU6050 mpu;

// ── MPU Calibration Offsets ──────────────────────────────────
float accelOffX = 0, accelOffY = 0, accelOffZ = 0;
float gyroOffX  = 0, gyroOffY  = 0;

// ── Vibration History ────────────────────────────────────────
float vibHistory[VIB_HISTORY];
uint8_t vibIdx = 0;

// ── Session Statistics ───────────────────────────────────────
struct Stats {
  float maxTemp,    minTemp;
  float maxHumid,   minHumid;
  float maxLevel,   minLevel;
  float maxTilt,    maxVib;
  uint32_t readings;
  uint32_t alerts;
};
Stats stats;

// ── System State ─────────────────────────────────────────────
unsigned long sessionStart  = 0;
unsigned long lastSample    = 0;
SystemState   systemState   = STATE_OK;
bool          alertsEnabled = true;
bool          mpuAvailable  = false;

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  dht.begin();
  Wire.begin();

  printBanner();
  initStats();

  // Initialise MPU
  mpu.initialize();
  if (mpu.testConnection()) {
    mpuAvailable = true;
    calibrateMPU();
  } else {
    Serial.println(F("  [WARN] MPU-6050 not detected — motion monitoring disabled."));
  }

  for (uint8_t i = 0; i < VIB_HISTORY; i++) vibHistory[i] = 0;

  sessionStart = millis();
  Serial.println(F("  All sensors ready. Field station online."));
  Serial.println(F("  Type 'h' for commands."));
  printDivider();
}

// ─────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  // Handle Serial commands
  if (Serial.available()) {
    handleCommand(Serial.read());
  }

  unsigned long now = millis();
  if (now - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = now;

  // ── Read all sensors ──────────────────────────────────────
  EnvData   env   = readEnvironmental();
  LevelData level = readLevel();
  MotionData motion = readMotion();

  stats.readings++;

  // ── Update statistics ─────────────────────────────────────
  if (env.valid)    updateEnvStats(env);
  if (level.valid)  updateLevelStats(level);
  if (motion.valid) updateMotionStats(motion);

  // ── Determine system state ────────────────────────────────
  systemState = evaluateSystemState(env, level, motion);

  // ── Print data row ────────────────────────────────────────
  unsigned long elapsed = (now - sessionStart) / 1000UL;
  printDataRow(elapsed, env, level, motion);

  // ── Print alerts if enabled ───────────────────────────────
  if (alertsEnabled) {
    printAlerts(env, level, motion);
  }

  // ── Print dashboard periodically ─────────────────────────
  if (stats.readings % DASHBOARD_INTERVAL == 0) {
    printDashboard(env, level, motion);
  }
}

// ─────────────────────────────────────────────────────────────
//  SENSOR READ FUNCTIONS
// ─────────────────────────────────────────────────────────────

EnvData readEnvironmental() {
  EnvData d;
  d.humidity    = dht.readHumidity();
  d.temperature = dht.readTemperature();
  d.valid       = !isnan(d.humidity) && !isnan(d.temperature);
  if (d.valid) {
    d.heatIndex = dht.computeHeatIndex(d.temperature, d.humidity, false);
  }
  return d;
}

LevelData readLevel() {
  LevelData d;
  // Take 3 samples, use median
  float samples[3];
  uint8_t valid = 0;
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long dur = pulseIn(ECHO_PIN, HIGH, 30000UL);
    if (dur > 0) samples[valid++] = (dur / 2.0) * 0.0343;
    delay(10);
  }
  if (valid == 0) { d.valid = false; return d; }
  // Sort and take median
  if (valid > 1 && samples[1] < samples[0]) { float t = samples[0]; samples[0] = samples[1]; samples[1] = t; }
  if (valid > 2 && samples[2] < samples[1]) { float t = samples[1]; samples[1] = samples[2]; samples[2] = t; }
  if (valid > 1 && samples[1] < samples[0]) { float t = samples[0]; samples[0] = samples[1]; samples[1] = t; }
  d.distanceCm = samples[valid / 2];
  d.distanceCm = constrain(d.distanceCm, SENSOR_OFFSET_CM, TANK_HEIGHT_CM);
  float usable = TANK_HEIGHT_CM - SENSOR_OFFSET_CM;
  d.levelPct   = constrain(((TANK_HEIGHT_CM - d.distanceCm) / usable) * 100.0, 0.0, 100.0);
  d.valid      = true;
  return d;
}

MotionData readMotion() {
  MotionData d;
  if (!mpuAvailable) { d.valid = false; return d; }

  int16_t rax, ray, raz, rgx, rgy, rgz;
  mpu.getMotion6(&rax, &ray, &raz, &rgx, &rgy, &rgz);

  float ax = (rax / ACCEL_SCALE) - accelOffX;
  float ay = (ray / ACCEL_SCALE) - accelOffY;
  float az = (raz / ACCEL_SCALE) - accelOffZ;

  d.roll      = atan2(ay, az) * 180.0 / PI;
  d.pitch     = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  float mag   = sqrt(ax*ax + ay*ay + az*az);
  float vib   = abs(mag - 1.0);

  vibHistory[vibIdx] = vib;
  vibIdx = (vibIdx + 1) % VIB_HISTORY;

  float sumSq = 0;
  for (uint8_t i = 0; i < VIB_HISTORY; i++) sumSq += vibHistory[i] * vibHistory[i];
  d.vibration = sqrt(sumSq / VIB_HISTORY);
  d.valid     = true;
  return d;
}

// ─────────────────────────────────────────────────────────────
//  STATE MACHINE
// ─────────────────────────────────────────────────────────────

SystemState evaluateSystemState(EnvData& e, LevelData& l, MotionData& m) {
  SystemState s = STATE_OK;

  if (e.valid) {
    if (e.temperature >= TEMP_HIGH || e.temperature <= TEMP_LOW) s = max(s, STATE_WARNING);
    if (e.humidity >= HUMIDITY_HIGH || e.humidity <= HUMIDITY_LOW) s = max(s, STATE_WARNING);
  }
  if (l.valid) {
    if (l.levelPct <= LEVEL_LOW || l.levelPct >= LEVEL_HIGH) s = max(s, STATE_WARNING);
    if (l.levelPct <= LEVEL_CRITICAL) s = max(s, STATE_CRITICAL);
  }
  if (m.valid) {
    float tilt = max(abs(m.roll), abs(m.pitch));
    if (tilt >= TILT_WARNING  || m.vibration >= VIB_WARNING)  s = max(s, STATE_WARNING);
    if (tilt >= TILT_CRITICAL || m.vibration >= VIB_CRITICAL) s = max(s, STATE_CRITICAL);
  }

  if (s > STATE_OK) stats.alerts++;
  return s;
}

SystemState max(SystemState a, SystemState b) {
  return (SystemState)(max((int)a, (int)b));
}

// ─────────────────────────────────────────────────────────────
//  DISPLAY FUNCTIONS
// ─────────────────────────────────────────────────────────────

void printDataRow(unsigned long sec, EnvData& e, LevelData& l, MotionData& m) {
  unsigned long h = sec/3600, mi = (sec%3600)/60, s = sec%60;
  char ts[12]; snprintf(ts, sizeof(ts), "%02lu:%02lu:%02lu", h, mi, s);

  const char* stateStr = (systemState == STATE_CRITICAL) ? "CRITICAL" :
                         (systemState == STATE_WARNING)  ? "WARNING " : "OK      ";

  Serial.print(F("["));   Serial.print(ts);        Serial.print(F("] "));
  Serial.print(F("T:"));
  if (e.valid) { Serial.print(e.temperature, 1); Serial.print(F("C ")); }
  else Serial.print(F("ERR  "));

  Serial.print(F("H:"));
  if (e.valid) { Serial.print(e.humidity, 1); Serial.print(F("% ")); }
  else Serial.print(F("ERR  "));

  Serial.print(F("Lv:"));
  if (l.valid) { Serial.print(l.levelPct, 1); Serial.print(F("% ")); }
  else Serial.print(F("ERR  "));

  if (m.valid) {
    Serial.print(F("Ro:"));  Serial.print(m.roll, 1);      Serial.print(F("° "));
    Serial.print(F("Vi:"));  Serial.print(m.vibration, 3); Serial.print(F("g "));
  } else {
    Serial.print(F("Motion:N/A "));
  }

  Serial.print(F("["));  Serial.print(stateStr);  Serial.print(F("]"));
  Serial.print(F("  #")); Serial.println(stats.readings);
}

void printAlerts(EnvData& e, LevelData& l, MotionData& m) {
  if (e.valid) {
    if (e.temperature >= TEMP_HIGH)  { Serial.print(F("  [!] High Temp: ")); Serial.print(e.temperature,1); Serial.println(F("C")); }
    if (e.temperature <= TEMP_LOW)   { Serial.print(F("  [!] Low Temp: "));  Serial.print(e.temperature,1); Serial.println(F("C")); }
    if (e.humidity >= HUMIDITY_HIGH) { Serial.print(F("  [!] High Humidity: ")); Serial.print(e.humidity,1); Serial.println(F("%")); }
    if (e.humidity <= HUMIDITY_LOW)  { Serial.print(F("  [!] Low Humidity: "));  Serial.print(e.humidity,1); Serial.println(F("%")); }
  }
  if (l.valid) {
    if (l.levelPct <= LEVEL_CRITICAL){ Serial.print(F("  [!!] CRITICAL Level: ")); Serial.print(l.levelPct,1); Serial.println(F("%")); }
    else if (l.levelPct <= LEVEL_LOW){ Serial.print(F("  [!] Low Level: ")); Serial.print(l.levelPct,1); Serial.println(F("%")); }
    if (l.levelPct >= LEVEL_HIGH)    { Serial.print(F("  [!] High Level: ")); Serial.print(l.levelPct,1); Serial.println(F("%")); }
  }
  if (m.valid) {
    float tilt = max(abs(m.roll), abs(m.pitch));
    if (tilt >= TILT_CRITICAL)       { Serial.print(F("  [!!] CRITICAL Tilt: ")); Serial.print(tilt,1); Serial.println(F("deg")); }
    else if (tilt >= TILT_WARNING)   { Serial.print(F("  [!] Tilt Warning: ")); Serial.print(tilt,1); Serial.println(F("deg")); }
    if (m.vibration >= VIB_CRITICAL) { Serial.print(F("  [!!] CRITICAL Vibration: ")); Serial.print(m.vibration,3); Serial.println(F("g")); }
    else if (m.vibration >= VIB_WARNING){ Serial.print(F("  [!] Vibration Warning: ")); Serial.print(m.vibration,3); Serial.println(F("g")); }
  }
}

void printDashboard(EnvData& e, LevelData& l, MotionData& m) {
  unsigned long uptime = (millis() - sessionStart) / 1000UL;
  unsigned long h = uptime/3600, mi = (uptime%3600)/60, s = uptime%60;

  const char* stateStr = (systemState == STATE_CRITICAL) ? "!! CRITICAL !!" :
                         (systemState == STATE_WARNING)  ? "*  WARNING  *"  : "    OK       ";

  Serial.println(F(""));
  Serial.println(F("  ╔════════════════════════════════════════════════╗"));
  Serial.println(F("  ║         FIELD STATION DASHBOARD                ║"));
  Serial.println(F("  ╠════════════════════════════════════════════════╣"));

  // Uptime and system state
  Serial.print  (F("  ║  Uptime  : "));
  char ts[12]; snprintf(ts, sizeof(ts), "%02lu:%02lu:%02lu", h, mi, s);
  Serial.print(ts);
  Serial.print  (F("          Readings: "));
  Serial.print  (stats.readings);
  Serial.println(F("        ║"));

  Serial.print  (F("  ║  System  : ["));
  Serial.print  (stateStr);
  Serial.print  (F("]   Alerts: "));
  Serial.print  (stats.alerts);
  Serial.println(F("               ║"));

  Serial.println(F("  ╠════════════════════════════════════════════════╣"));

  // Environmental
  Serial.println(F("  ║  [ENVIRONMENTAL — DHT11]                       ║"));
  if (e.valid) {
    Serial.print  (F("  ║    Temperature : "));
    Serial.print  (e.temperature, 1);
    Serial.print  (F(" C   (max: "));
    Serial.print  (stats.maxTemp, 1);
    Serial.print  (F(" / min: "));
    Serial.print  (stats.minTemp, 1);
    Serial.println(F(" C)          ║"));
    Serial.print  (F("  ║    Humidity    : "));
    Serial.print  (e.humidity, 1);
    Serial.print  (F(" %   (max: "));
    Serial.print  (stats.maxHumid, 1);
    Serial.print  (F(" / min: "));
    Serial.print  (stats.minHumid, 1);
    Serial.println(F(" %)          ║"));
    Serial.print  (F("  ║    Heat Index  : "));
    Serial.print  (e.heatIndex, 1);
    Serial.println(F(" C                            ║"));
  } else {
    Serial.println(F("  ║    [SENSOR ERROR — check wiring]               ║"));
  }

  Serial.println(F("  ╠════════════════════════════════════════════════╣"));

  // Level
  Serial.println(F("  ║  [LEVEL MONITOR — HC-SR04]                     ║"));
  if (l.valid) {
    Serial.print  (F("  ║    Distance    : "));
    Serial.print  (l.distanceCm, 1);
    Serial.println(F(" cm                               ║"));
    Serial.print  (F("  ║    Fill Level  : "));
    Serial.print  (l.levelPct, 1);
    Serial.print  (F(" %  "));
    printBarInline(l.levelPct, 15);
    Serial.println(F("    ║"));
    Serial.print  (F("  ║    Range       : max "));
    Serial.print  (stats.maxLevel, 1);
    Serial.print  (F("% / min "));
    Serial.print  (stats.minLevel, 1);
    Serial.println(F("%               ║"));
  } else {
    Serial.println(F("  ║    [SENSOR ERROR — check wiring]               ║"));
  }

  Serial.println(F("  ╠════════════════════════════════════════════════╣"));

  // Motion
  Serial.println(F("  ║  [MOTION & TILT — MPU-6050]                    ║"));
  if (m.valid) {
    Serial.print  (F("  ║    Roll        : "));
    Serial.print  (m.roll, 1);
    Serial.println(F(" deg                              ║"));
    Serial.print  (F("  ║    Pitch       : "));
    Serial.print  (m.pitch, 1);
    Serial.println(F(" deg                              ║"));
    Serial.print  (F("  ║    Vibration   : "));
    Serial.print  (m.vibration, 3);
    Serial.print  (F(" g   (peak: "));
    Serial.print  (stats.maxVib, 3);
    Serial.println(F(" g)           ║"));
    Serial.print  (F("  ║    Max Tilt    : "));
    Serial.print  (stats.maxTilt, 1);
    Serial.println(F(" deg                              ║"));
  } else {
    Serial.println(F("  ║    [NOT AVAILABLE]                             ║"));
  }

  Serial.println(F("  ╚════════════════════════════════════════════════╝"));
  Serial.println(F("  Commands: 's'=status  'r'=reset  'a'=alerts  'h'=help"));
  Serial.println(F(""));
}

void printBarInline(float level, uint8_t width) {
  uint8_t filled = (uint8_t)((level / 100.0) * width);
  Serial.print(F("["));
  for (uint8_t i = 0; i < width; i++) {
    Serial.print(i < filled ? F("█") : F("░"));
  }
  Serial.print(F("]"));
}

// ─────────────────────────────────────────────────────────────
//  COMMAND INTERFACE
// ─────────────────────────────────────────────────────────────

void handleCommand(char cmd) {
  switch (cmd) {
    case 's': case 'S': {
      EnvData e = readEnvironmental();
      LevelData l = readLevel();
      MotionData m = readMotion();
      printDashboard(e, l, m);
      break;
    }
    case 'r': case 'R':
      initStats();
      Serial.println(F("[CMD] Session statistics reset."));
      break;
    case 'a': case 'A':
      alertsEnabled = !alertsEnabled;
      Serial.print(F("[CMD] Alerts "));
      Serial.println(alertsEnabled ? F("ENABLED.") : F("DISABLED."));
      break;
    case 'h': case 'H':
      printHelp();
      break;
    default:
      break;
  }
}

void printHelp() {
  Serial.println(F(""));
  Serial.println(F("  ┌─────────────────────────────────┐"));
  Serial.println(F("  │      AVAILABLE COMMANDS          │"));
  Serial.println(F("  │  s — Print status dashboard      │"));
  Serial.println(F("  │  r — Reset session statistics    │"));
  Serial.println(F("  │  a — Toggle alert messages       │"));
  Serial.println(F("  │  h — Show this help menu         │"));
  Serial.println(F("  └─────────────────────────────────┘"));
}

// ─────────────────────────────────────────────────────────────
//  STATISTICS
// ─────────────────────────────────────────────────────────────

void initStats() {
  stats.maxTemp  = -999; stats.minTemp  =  999;
  stats.maxHumid = -999; stats.minHumid =  999;
  stats.maxLevel = -999; stats.minLevel =  999;
  stats.maxTilt  = 0;    stats.maxVib   = 0;
  stats.readings = 0;    stats.alerts   = 0;
}

void updateEnvStats(EnvData& e) {
  if (e.temperature > stats.maxTemp)  stats.maxTemp  = e.temperature;
  if (e.temperature < stats.minTemp)  stats.minTemp  = e.temperature;
  if (e.humidity    > stats.maxHumid) stats.maxHumid = e.humidity;
  if (e.humidity    < stats.minHumid) stats.minHumid = e.humidity;
}

void updateLevelStats(LevelData& l) {
  if (l.levelPct > stats.maxLevel) stats.maxLevel = l.levelPct;
  if (l.levelPct < stats.minLevel) stats.minLevel = l.levelPct;
}

void updateMotionStats(MotionData& m) {
  float tilt = max(abs(m.roll), abs(m.pitch));
  if (tilt        > stats.maxTilt) stats.maxTilt = tilt;
  if (m.vibration > stats.maxVib)  stats.maxVib  = m.vibration;
}

// ─────────────────────────────────────────────────────────────
//  MPU CALIBRATION
// ─────────────────────────────────────────────────────────────

void calibrateMPU() {
  Serial.print(F("  Calibrating MPU-6050 (keep still)"));
  long sAx=0, sAy=0, sAz=0, sGx=0, sGy=0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    sAx+=ax; sAy+=ay; sAz+=az; sGx+=gx; sGy+=gy;
    if (i % 30 == 0) Serial.print(F("."));
    delay(5);
  }
  accelOffX = (sAx/CALIB_SAMPLES)/ACCEL_SCALE;
  accelOffY = (sAy/CALIB_SAMPLES)/ACCEL_SCALE;
  accelOffZ = ((sAz/CALIB_SAMPLES)/ACCEL_SCALE) - 1.0;
  gyroOffX  = (sGx/CALIB_SAMPLES)/GYRO_SCALE;
  gyroOffY  = (sGy/CALIB_SAMPLES)/GYRO_SCALE;
  Serial.println(F(" Done."));
}

// ─────────────────────────────────────────────────────────────
//  STARTUP
// ─────────────────────────────────────────────────────────────

void printBanner() {
  Serial.println(F(""));
  Serial.println(F("  ╔══════════════════════════════════════════════════╗"));
  Serial.println(F("  ║        MULTI-SENSOR FIELD STATION  v1.0          ║"));
  Serial.println(F("  ║   DHT11  |  HC-SR04  |  MPU-6050                 ║"));
  Serial.println(F("  ║   Environment | Level | Motion                    ║"));
  Serial.println(F("  ╚══════════════════════════════════════════════════╝"));
  Serial.println(F("  Initialising sensors..."));
  delay(2000);
}

void printDivider() {
  Serial.println(F("  ──────────────────────────────────────────────────────"));
}
