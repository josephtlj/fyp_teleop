// Feather nRF52840 Sense: orientation streamer for the teleop IMUs.
// Fuses accel/gyro/mag and sends one line per update over BLE UART:
//   seq,qw,qx,qy,qz\n
// The same line plus roll/pitch/yaw (degrees) is printed on USB serial for debugging.
// Madgwick, Mahony, the NXP Kalman filter and two extra Madgwick copies (for tuning beta) all
// run on the same samples every update, so the active one can be switched at any time and
// any three can be compared fairly.
// Each board gets its own IMU_ID (1, 2 or 3), which sets its BLE name FYP-IMU-<id>.
//
// Commands received over BLE UART (one per line):
//   raw                     toggle raw magnetometer mode: lines become seq,mx,my,mz (uT, uncorrected).
//                           the red LED and a blue NeoPixel stay on while it is active, so the
//                           person rotating the board knows when to keep going
//   cal,ox,oy,oz,sx,sy,sz   store magnetometer calibration: corrected = (raw - o) * s
//   cal?                    reply with the stored calibration
//   bat?                    reply with bat,<millivolts>
// The NeoPixel glows red while the battery is low (below BAT_LOW_MV, cleared above BAT_OK_MV).
//   filter,<name>           stream from madgwick, mahony, kalman, madgwick_b or madgwick_c
//   beta,<name>,<value>     set beta of one of the Madgwick filters
//   compare[,<a>,<b>,<c>]   toggle compare mode (optionally choosing the three filters, default
//                           madgwick,mahony,kalman): lines become seq then roll,pitch,yaw of each
//                           chosen filter, in tenths of a degree, at COMPARE_HZ
// The calibration is kept in internal flash, so each board keeps its own across power cycles.

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_NeoPixel.h>

// Override per board at build time: --build-property compiler.cpp.extra_flags=-DIMU_ID=2
#ifndef IMU_ID
#define IMU_ID 1
#endif

const float FILTER_HZ = 100.0f;  // fusion update rate
const float SEND_HZ = 50.0f;     // BLE send rate
const float COMPARE_HZ = 25.0f;  // BLE send rate in compare mode (lines are longer)
const int GYRO_BIAS_SAMPLES = 200;
const uint32_t BAT_CHECK_MS = 5000;
const int BAT_LOW_MV = 3500;  // about 10-15% left on a LiPo
const int BAT_OK_MV = 3600;   // hysteresis so the warning doesn't flicker

Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

Adafruit_LSM6DS33 lsm6ds33;
Adafruit_LSM6DS3TRC lsm6ds3trc;
Adafruit_LSM6DS *imu = nullptr;
Adafruit_LIS3MDL mag;
Adafruit_Madgwick madgwick, madgwickB, madgwickC;
Adafruit_Mahony mahony;
Adafruit_NXPSensorFusion kalman;
const int NUM_FILTERS = 5;
Adafruit_AHRS_FusionInterface *filters[NUM_FILTERS] = {&madgwick, &mahony, &kalman, &madgwickB, &madgwickC};
const char *FILTER_NAMES[NUM_FILTERS] = {"madgwick", "mahony", "kalman", "madgwick_b", "madgwick_c"};
Adafruit_Madgwick *madgwicks[NUM_FILTERS] = {&madgwick, nullptr, nullptr, &madgwickB, &madgwickC};
const int NUM_COMPARED = 3;
int compared[NUM_COMPARED] = {0, 1, 2};
BLEUart bleuart;

using namespace Adafruit_LittleFS_Namespace;
const char *CAL_FILE = "/magcal";

float gyroBias[3] = {0, 0, 0};
float magOffset[3] = {0, 0, 0};
float magScale[3] = {1, 1, 1};
bool rawMode = false;
bool compareMode = false;
bool batteryLow = false;
int active = 0;  // index into filters
uint32_t updateMicros = 0;  // time taken by the last sensor read + all filter updates
uint32_t seq = 0;

void loadMagCal() {
  File f(InternalFS);
  if (!f.open(CAL_FILE, FILE_O_READ)) return;
  float v[6];
  if (f.read(v, sizeof(v)) == sizeof(v)) {
    memcpy(magOffset, v, sizeof(magOffset));
    memcpy(magScale, v + 3, sizeof(magScale));
  }
  f.close();
}

void saveMagCal() {
  // FILE_O_WRITE appends, so remove the old file first.
  InternalFS.remove(CAL_FILE);
  File f(InternalFS);
  if (!f.open(CAL_FILE, FILE_O_WRITE)) return;
  float v[6];
  memcpy(v, magOffset, sizeof(magOffset));
  memcpy(v + 3, magScale, sizeof(magScale));
  f.write((uint8_t *)v, sizeof(v));
  f.close();
}

void replyMagCal() {
  char line[96];
  int n = snprintf(line, sizeof(line), "cal,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n",
                   magOffset[0], magOffset[1], magOffset[2],
                   magScale[0], magScale[1], magScale[2]);
  bleuart.write((uint8_t *)line, n);
}

int filterIndex(const char *name) {
  for (int i = 0; i < NUM_FILTERS; i++) {
    if (strcmp(name, FILTER_NAMES[i]) == 0) return i;
  }
  return -1;
}

// VBAT goes through a 1:2 divider to A6. Read it against the 3.0 V internal reference.
int readBatteryMv() {
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  delay(1);
  int raw = analogRead(PIN_VBAT);
  analogReference(AR_DEFAULT);
  analogReadResolution(10);
  return raw * 3000 * 2 / 4096;
}

void checkBattery() {
  static uint32_t last = 0;
  if (millis() - last < BAT_CHECK_MS) return;
  last = millis();
  int mv = readBatteryMv();
  bool low = batteryLow ? mv < BAT_OK_MV : mv < BAT_LOW_MV;
  if (low != batteryLow && !rawMode) {
    batteryLow = low;
    pixel.setPixelColor(0, low ? pixel.Color(40, 0, 0) : 0);
    pixel.show();
  }
}

void handleCommand(char *cmd) {
  if (strcmp(cmd, "raw") == 0) {
    rawMode = !rawMode;
    digitalWrite(LED_RED, rawMode ? HIGH : LOW);
    // The NeoPixel is far easier to see than the red LED: blue while recording.
    pixel.setPixelColor(0, rawMode ? pixel.Color(0, 0, 120) : batteryLow ? pixel.Color(40, 0, 0) : 0);
    pixel.show();
  } else if (strcmp(cmd, "compare") == 0) {
    compareMode = !compareMode;
  } else if (strncmp(cmd, "compare,", 8) == 0) {
    int chosen[NUM_COMPARED];
    char *name = strtok(cmd + 8, ",");
    for (int i = 0; i < NUM_COMPARED; i++, name = strtok(nullptr, ",")) {
      if (!name || (chosen[i] = filterIndex(name)) < 0) return;
    }
    memcpy(compared, chosen, sizeof(compared));
    compareMode = true;
  } else if (strncmp(cmd, "filter,", 7) == 0) {
    int i = filterIndex(cmd + 7);
    if (i >= 0) active = i;
  } else if (strncmp(cmd, "beta,", 5) == 0) {
    char *name = strtok(cmd + 5, ",");
    char *value = strtok(nullptr, ",");
    int i = name ? filterIndex(name) : -1;
    if (i >= 0 && madgwicks[i] && value) madgwicks[i]->setBeta(strtof(value, nullptr));
  } else if (strcmp(cmd, "bat?") == 0) {
    char line[24];
    int n = snprintf(line, sizeof(line), "bat,%d\n", readBatteryMv());
    bleuart.write((uint8_t *)line, n);
  } else if (strcmp(cmd, "cal?") == 0) {
    replyMagCal();
  } else if (strncmp(cmd, "cal,", 4) == 0) {
    // strtof, not sscanf: the nano C library's scanf has no float support.
    float v[6];
    char *p = cmd + 4, *end;
    for (int i = 0; i < 6; i++) {
      v[i] = strtof(p, &end);
      if (end == p || (i < 5 && *end != ',')) return;
      p = end + 1;
    }
    memcpy(magOffset, v, sizeof(magOffset));
    memcpy(magScale, v + 3, sizeof(magScale));
    saveMagCal();
    replyMagCal();
  }
}

void readCommands() {
  static char cmd[96];
  static int len = 0;
  while (bleuart.available()) {
    char c = bleuart.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        cmd[len] = 0;
        handleCommand(cmd);
        len = 0;
      }
    } else if (len < (int)sizeof(cmd) - 1) {
      cmd[len++] = c;
    }
  }
}

void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void halt(const char *msg) {
  while (true) {
    Serial.println(msg);
    delay(1000);
  }
}

// Averages the gyro while the board is held still, so the filter starts without drift.
void calibrateGyro() {
  sensors_event_t a, g, t;
  for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
    imu->getEvent(&a, &g, &t);
    gyroBias[0] += g.gyro.x;
    gyroBias[1] += g.gyro.y;
    gyroBias[2] += g.gyro.z;
    delay(5);
  }
  for (int i = 0; i < 3; i++) gyroBias[i] /= GYRO_BIAS_SAMPLES;
}

// A reset in the middle of an I2C read (e.g. reflashing while the battery keeps the sensors
// powered) can leave a sensor holding SDA low. Clocking SCL lets it finish and release the bus.
void recoverI2C() {
  pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
  pinMode(PIN_WIRE_SCL, OUTPUT);
  for (int i = 0; i < 9 && digitalRead(PIN_WIRE_SDA) == LOW; i++) {
    digitalWrite(PIN_WIRE_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_WIRE_SCL, HIGH);
    delayMicroseconds(10);
  }
  // STOP condition: SDA low -> high while SCL is high.
  pinMode(PIN_WIRE_SDA, OUTPUT);
  digitalWrite(PIN_WIRE_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_WIRE_SDA, HIGH);
  delayMicroseconds(10);
  pinMode(PIN_WIRE_SDA, INPUT);
  pinMode(PIN_WIRE_SCL, INPUT);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);

  recoverI2C();
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);
  pixel.begin();
  pixel.clear();
  pixel.show();

  // Board revisions ship with either an LSM6DS33 or an LSM6DS3TR-C.
  if (lsm6ds33.begin_I2C()) imu = &lsm6ds33;
  else if (lsm6ds3trc.begin_I2C()) imu = &lsm6ds3trc;
  else halt("No LSM6DS accel/gyro found");
  if (!mag.begin_I2C()) halt("No LIS3MDL magnetometer found");

  imu->setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  // 500 dps saturated during fast waving, leaving the heading off for several seconds.
  imu->setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  imu->setAccelDataRate(LSM6DS_RATE_208_HZ);
  imu->setGyroDataRate(LSM6DS_RATE_208_HZ);
  mag.setPerformanceMode(LIS3MDL_HIGHMODE);
  mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
  mag.setRange(LIS3MDL_RANGE_4_GAUSS);

  InternalFS.begin();
  loadMagCal();

  Serial.println(imu == &lsm6ds33 ? "IMU: LSM6DS33" : "IMU: LSM6DS3TR-C");
  Serial.println("Calibrating gyro, keep still...");
  calibrateGyro();
  for (int i = 0; i < NUM_FILTERS; i++) filters[i]->begin(FILTER_HZ);
  // Library default is 0.1. 0.5 settled ~10x faster after fast motion (0.6s vs 6.3s) with only
  // ~0.02 deg more sample-to-sample jitter (recording: fyp_data/imu/beta_compare_1.csv).
  madgwick.setBeta(0.5f);

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  char name[16];
  snprintf(name, sizeof(name), "FYP-IMU-%d", IMU_ID);
  Bluefruit.setName(name);
  bleuart.begin();
  startAdvertising();
  Serial.print("Advertising as ");
  Serial.println(name);
}

void loop() {
  readCommands();
  checkBattery();

  // Advance by a fixed period (not to `now`) so the rate doesn't slip below target.
  static uint32_t lastFilter = micros(), tick = 0;
  const uint32_t period = 1e6 / FILTER_HZ;
  if (micros() - lastFilter < period) return;
  lastFilter += period;

  uint32_t start = micros();
  sensors_event_t a, g, t, m;
  imu->getEvent(&a, &g, &t);
  mag.getEvent(&m);
  float mx = (m.magnetic.x - magOffset[0]) * magScale[0];
  float my = (m.magnetic.y - magOffset[1]) * magScale[1];
  float mz = (m.magnetic.z - magOffset[2]) * magScale[2];
  // Adafruit AHRS takes gyro in deg/s, accel in g (the Kalman filter needs g) and mag in uT.
  for (int i = 0; i < NUM_FILTERS; i++) {
    filters[i]->update((g.gyro.x - gyroBias[0]) * SENSORS_RADS_TO_DPS,
                       (g.gyro.y - gyroBias[1]) * SENSORS_RADS_TO_DPS,
                       (g.gyro.z - gyroBias[2]) * SENSORS_RADS_TO_DPS,
                       a.acceleration.x / SENSORS_GRAVITY_STANDARD,
                       a.acceleration.y / SENSORS_GRAVITY_STANDARD,
                       a.acceleration.z / SENSORS_GRAVITY_STANDARD,
                       mx, my, mz);
  }
  updateMicros = micros() - start;

  tick++;
  char line[96];
  int n;
  if (compareMode) {
    if (tick % (int)(FILTER_HZ / COMPARE_HZ) != 0) return;
    n = snprintf(line, sizeof(line), "%lu", (unsigned long)seq++);
    for (int i = 0; i < NUM_COMPARED; i++) {
      Adafruit_AHRS_FusionInterface *f = filters[compared[i]];
      n += snprintf(line + n, sizeof(line) - n, ",%d,%d,%d",
                    (int)lroundf(f->getRoll() * 10),
                    (int)lroundf(f->getPitch() * 10),
                    (int)lroundf(f->getYaw() * 10));
    }
    n += snprintf(line + n, sizeof(line) - n, "\n");
    if (Bluefruit.connected()) bleuart.write((uint8_t *)line, n);
    Serial.printf("%.*s,us=%lu\n", n - 1, line, (unsigned long)updateMicros);
    return;
  }

  if (tick % (int)(FILTER_HZ / SEND_HZ) != 0) return;

  if (rawMode) {
    n = snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.2f\n",
                 (unsigned long)seq++, m.magnetic.x, m.magnetic.y, m.magnetic.z);
    if (Bluefruit.connected()) bleuart.write((uint8_t *)line, n);
    return;
  }

  Adafruit_AHRS_FusionInterface *filter = filters[active];
  float qw, qx, qy, qz;
  filter->getQuaternion(&qw, &qx, &qy, &qz);
  n = snprintf(line, sizeof(line), "%lu,%.4f,%.4f,%.4f,%.4f\n",
               (unsigned long)seq++, qw, qx, qy, qz);
  if (Bluefruit.connected()) bleuart.write((uint8_t *)line, n);

  Serial.printf("%.*s,rpy=%.1f,%.1f,%.1f,%s,us=%lu\n", n - 1, line,
                filter->getRoll(), filter->getPitch(), filter->getYaw(),
                FILTER_NAMES[active], (unsigned long)updateMicros);
}
