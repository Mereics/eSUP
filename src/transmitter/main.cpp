#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "display_gc9a01.h"
#include "../common/remote_protocol.h"

constexpr int HALL_PIN = 6;
constexpr int IMU_SDA_PIN = 7;
constexpr int IMU_SCL_PIN = 8;
constexpr int SPEED_HOLD_TOUCH_PIN = 9;
constexpr int ARM_TOUCH_PIN = 10;
constexpr int REVERSE_TOUCH_PIN = 11;
constexpr int COURSE_HOLD_TOUCH_PIN = 12;
constexpr int BATTERY_SENSE_PIN = 13;
constexpr uint8_t MPU_ADDR = 0x68;

constexpr uint16_t ADC_MAX_VALUE = 4095;
constexpr float ADC_REF_VOLTAGE = 3.3f;

constexpr uint16_t FRAME_INTERVAL_MS = 80;
constexpr uint16_t SENSOR_INTERVAL_MS = 15;
constexpr uint16_t HALL_BOOT_CALIBRATION_MS = 700;
constexpr uint16_t HALL_MIN_SPAN_RAW = 700;
constexpr uint16_t HALL_NOISE_DEADBAND_RAW = 25;
constexpr uint16_t HALL_START_LEARN_THRESHOLD_RAW = 80;
constexpr uint8_t MOTOR_START_PERCENT = 15;
constexpr float BATTERY_DIVIDER_RATIO = 2.0f;
constexpr uint16_t BATTERY_SAMPLE_INTERVAL_MS = 250;
constexpr bool SERIAL_DEBUG_ENABLED = false;
constexpr bool TX_BATTERY_MONITOR_ENABLED = false;
constexpr uint16_t ARM_HOLD_MS = 2000;
constexpr uint16_t DIRECTION_CHANGE_NEUTRAL_MS = 1000;
constexpr uint16_t TELEMETRY_FAILSAFE_MS = 500;
constexpr uint16_t TELEMETRY_STALE_MS = 1000;
constexpr uint16_t BATTERY_FAILSAFE_CONFIRM_MS = 500;
constexpr float MAIN_BATTERY_DISARM_VOLTAGE = 21.0f;
constexpr float TX_BATTERY_DISARM_VOLTAGE = 3.4f;
constexpr int8_t WIFI_MAX_TX_POWER = 78; // 19.5 dBm in 0.25 dBm units.

// Calibrare throttle. Pentru moment folosim rest auto la boot si min/max live.
// Valorile negative fata de rest sunt fortate la 0.
uint16_t hallReleasedRaw = 0;
uint16_t hallMaxRaw = 0;
float hallFilteredRaw = 0.0f;
uint8_t throttlePercent = 0;
uint8_t throttleCommandPercent = 0;
bool hallMaxLearned = false;

// Calibrare steering din accelerometrul MPU.
// Montaj curent in maner: Y in sus, Z spre dreapta, X spre fata.
// Roll stanga/dreapta este rotatia in jurul axei X, deci il calculam din Y/Z.
constexpr bool STEERING_INVERT = true;
constexpr float STEERING_MOUNT_OFFSET_DEG = 0.0f;
constexpr float STEERING_DEADZONE_DEG = 4.0f;
constexpr float STEERING_FULL_SCALE_DEG = 35.0f;
float imuRollOffsetDeg = STEERING_MOUNT_OFFSET_DEG;
float imuRollFilteredDeg = 0.0f;
int8_t steeringPercent = 0;
bool imuOk = false;
uint8_t imuConsecutiveErrors = 0;
uint32_t lastGoodImuMs = 0;
float lastAx = 0.0f;
float lastAy = 0.0f;
float lastAz = 0.0f;

bool armed = false;
bool armRequested = false;
uint8_t armToggleCount = 0;
bool lastArmTouchReading = false;
bool stableArmTouchState = false;
uint32_t lastArmTouchChangeMs = 0;
uint32_t armPressStartedMs = 0;
bool armPressHandled = false;
constexpr uint16_t TOUCH_DEBOUNCE_MS = 60;

bool cruiseActive = false;
uint8_t cruiseThrottlePercent = 0;
bool lastCruiseTouchReading = false;
bool stableCruiseTouchState = false;
uint32_t lastCruiseTouchChangeMs = 0;
bool lastCourseTouchReading = false;
bool stableCourseTouchState = false;
uint32_t lastCourseTouchChangeMs = 0;
bool lastReverseTouchReading = false;
bool stableReverseTouchState = false;
uint32_t lastReverseTouchChangeMs = 0;
uint32_t throttleNeutralSinceMs = 0;

float batteryVoltage = 0.0f;
uint8_t transmitterBatteryPercent = 100;
uint32_t lastBatterySampleMs = 0;
uint32_t mainBatteryLowSinceMs = 0;
uint32_t txBatteryLowSinceMs = 0;
bool telemetryFailsafeActive = false;
bool mainBatteryFailsafeActive = false;
bool txBatteryFailsafeActive = false;
bool courseHoldActive = false;
bool reverseActive = false;
float courseTargetHeading = 0.0f;

uint16_t controlSeq = 0;
uint16_t telemetrySeq = 0;
uint32_t lastControlSendMs = 0;
uint32_t lastTelemetryMs = 0;
uint8_t linkQuality = 0;
uint8_t telemetrySatellites = 0;
uint8_t telemetryMainBattery = 100;
uint8_t telemetryRemoteBattery = 100;
float telemetrySpeed = 0.0f;
float telemetryHeading = 0.0f;
float telemetryBatteryVoltage = 0.0f;
int8_t telemetryRssi = 0;
constexpr uint8_t RECEIVER_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint16_t CONTROL_SEND_INTERVAL_MS = 40;

template <typename... Args>
void serialPrintf(const char *format, Args... args) {
  if (SERIAL_DEBUG_ENABLED && Serial) {
    Serial.printf(format, args...);
  }
}

void serialPrintln(const char *message) {
  if (SERIAL_DEBUG_ENABLED && Serial) {
    Serial.println(message);
  }
}

uint16_t bg;
uint16_t panel;
uint16_t track;
uint16_t textDim;
uint16_t textMain;
uint16_t cyan;
uint16_t green;
uint16_t orange;
uint16_t yellow;
uint16_t red;

uint16_t readHallRaw() {
  uint32_t sum = 0;
  constexpr uint8_t samples = 16;

  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(HALL_PIN);
    delayMicroseconds(200);
  }

  return sum / samples;
}

uint16_t readAdcAveraged(int pin) {
  uint32_t sum = 0;
  constexpr uint8_t samples = 16;

  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }

  return sum / samples;
}

float rawToVoltage(float raw) {
  return raw * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
}

uint8_t batteryPercentFromVoltage(float voltage) {
  if (voltage >= 4.20f) return 100;
  if (voltage >= 4.10f) return 90;
  if (voltage >= 4.00f) return 80;
  if (voltage >= 3.90f) return 70;
  if (voltage >= 3.80f) return 55;
  if (voltage >= 3.70f) return 40;
  if (voltage >= 3.60f) return 25;
  if (voltage >= 3.50f) return 15;
  if (voltage >= 3.40f) return 8;
  if (voltage >= 3.30f) return 3;
  return 0;
}

void updateBatteryVoltage() {
  const uint16_t raw = readAdcAveraged(BATTERY_SENSE_PIN);
  const float measured = rawToVoltage(raw) * BATTERY_DIVIDER_RATIO;

  if (batteryVoltage <= 0.01f) {
    batteryVoltage = measured;
  } else {
    batteryVoltage = batteryVoltage * 0.92f + measured * 0.08f;
  }

  transmitterBatteryPercent = batteryPercentFromVoltage(batteryVoltage);
}

void updateThrottle() {
  const uint16_t raw = readHallRaw();
  hallFilteredRaw = hallFilteredRaw * 0.88f + raw * 0.12f;

  const float delta = hallFilteredRaw - hallReleasedRaw;
  if (delta > HALL_START_LEARN_THRESHOLD_RAW && hallFilteredRaw > hallMaxRaw) {
    hallMaxRaw = static_cast<uint16_t>(hallFilteredRaw + 0.5f);
    hallMaxLearned = true;
  }

  const float span = max(static_cast<float>(HALL_MIN_SPAN_RAW),
                         static_cast<float>(hallMaxRaw - hallReleasedRaw));
  const float positiveTravel = delta > HALL_NOISE_DEADBAND_RAW
                                   ? delta - HALL_NOISE_DEADBAND_RAW
                                   : 0.0f;
  throttlePercent = static_cast<uint8_t>(
      constrain(positiveTravel * 100.0f / span, 0.0f, 100.0f));
}

uint8_t motorMappedThrottle(uint8_t inputPercent) {
  if (inputPercent == 0) {
    return 0;
  }

  return static_cast<uint8_t>(
      map(inputPercent, 0, 100, MOTOR_START_PERCENT, 100));
}

void updateThrottleCommand() {
  if (telemetryFailsafeActive || mainBatteryFailsafeActive ||
      txBatteryFailsafeActive) {
    throttleCommandPercent = 0;
    return;
  }

  const uint8_t liveCommand = motorMappedThrottle(throttlePercent);
  throttleCommandPercent = cruiseActive ? cruiseThrottlePercent : liveCommand;
}

uint16_t calibrateHallReleased() {
  const uint32_t start = millis();
  uint32_t sum = 0;
  uint16_t count = 0;

  while (millis() - start < HALL_BOOT_CALIBRATION_MS) {
    sum += readHallRaw();
    count++;
    delay(5);
  }

  return count > 0 ? sum / count : readHallRaw();
}

bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadBytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t read = Wire.requestFrom(MPU_ADDR, length);
  if (read != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool initMpu() {
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, 400000);
  Wire.setTimeOut(50);
  delay(100);

  uint8_t whoAmI = 0;
  if (!mpuReadBytes(0x75, &whoAmI, 1)) {
    return false;
  }

  // MPU-9250/9255: 0x71/0x73, MPU-6500: 0x70.
  if (whoAmI != 0x70 && whoAmI != 0x71 && whoAmI != 0x73) {
    serialPrintf("MPU WHO_AM_I neasteptat: 0x%02X\n", whoAmI);
  } else {
    serialPrintf("MPU WHO_AM_I: 0x%02X\n", whoAmI);
  }

  if (!mpuWrite(0x6B, 0x00)) {
    return false;
  }
  delay(20);
  mpuWrite(0x1A, 0x04); // low-pass filter
  mpuWrite(0x1C, 0x00); // accel +/-2g
  mpuWrite(0x1B, 0x00); // gyro +/-250 dps
  return true;
}

bool readAccel(float &ax, float &ay, float &az) {
  uint8_t data[6] = {};
  if (!mpuReadBytes(0x3B, data, sizeof(data))) {
    return false;
  }

  const int16_t rawX = static_cast<int16_t>((data[0] << 8) | data[1]);
  const int16_t rawY = static_cast<int16_t>((data[2] << 8) | data[3]);
  const int16_t rawZ = static_cast<int16_t>((data[4] << 8) | data[5]);

  ax = rawX / 16384.0f;
  ay = rawY / 16384.0f;
  az = rawZ / 16384.0f;

  const float magnitude = sqrtf(ax * ax + ay * ay + az * az);
  if (magnitude < 0.55f || magnitude > 1.45f) {
    return false;
  }

  return true;
}

float readSteeringTiltDeg() {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 1.0f;
  if (!readAccel(ax, ay, az)) {
    imuConsecutiveErrors++;
    if (imuConsecutiveErrors > 5 || millis() - lastGoodImuMs > 300) {
      imuOk = false;
      steeringPercent = 0;
    }
    return imuRollFilteredDeg;
  }

  imuOk = true;
  imuConsecutiveErrors = 0;
  lastGoodImuMs = millis();
  lastAx = ax;
  lastAy = ay;
  lastAz = az;
  return atan2f(az, ay) * RAD_TO_DEG;
}

void calibrateImuRest() {
  constexpr uint8_t samples = 40;
  float sum = 0.0f;
  uint8_t okSamples = 0;

  for (uint8_t i = 0; i < samples; i++) {
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 1.0f;
    if (readAccel(ax, ay, az)) {
      sum += atan2f(az, ay) * RAD_TO_DEG;
      okSamples++;
    }
    delay(10);
  }

  if (okSamples > 0) {
    imuRollOffsetDeg = sum / okSamples;
    imuRollFilteredDeg = 0.0f;
  }
}

void updateSteering() {
  const float roll = readSteeringTiltDeg() - imuRollOffsetDeg;
  if (!imuOk) {
    imuRollFilteredDeg *= 0.90f;
    if (abs(imuRollFilteredDeg) < 0.5f) {
      imuRollFilteredDeg = 0.0f;
    }
    steeringPercent = 0;
    return;
  }

  float wrappedRoll = roll;
  while (wrappedRoll > 180.0f) {
    wrappedRoll -= 360.0f;
  }
  while (wrappedRoll < -180.0f) {
    wrappedRoll += 360.0f;
  }

  imuRollFilteredDeg = imuRollFilteredDeg * 0.86f + wrappedRoll * 0.14f;

  float steering = imuRollFilteredDeg;
  if (abs(steering) < STEERING_DEADZONE_DEG) {
    steering = 0.0f;
  } else if (steering > 0.0f) {
    steering -= STEERING_DEADZONE_DEG;
  } else {
    steering += STEERING_DEADZONE_DEG;
  }

  const float usableScale = STEERING_FULL_SCALE_DEG - STEERING_DEADZONE_DEG;
  steering = constrain(steering * 100.0f / usableScale, -100.0f, 100.0f);
  if (STEERING_INVERT) {
    steering = -steering;
  }

  steeringPercent = static_cast<int8_t>(steering);
}

bool telemetryIsFresh(uint32_t now, uint32_t timeoutMs) {
  return lastTelemetryMs != 0 && now - lastTelemetryMs <= timeoutMs;
}

bool localFailsafeActive() {
  return telemetryFailsafeActive || mainBatteryFailsafeActive ||
         txBatteryFailsafeActive;
}

void requestArmState(bool requested, bool forceSend = false) {
  if (armRequested == requested && !forceSend) {
    return;
  }

  armRequested = requested;
  armToggleCount++;
}

void enterLocalFailsafe(bool &failsafeFlag) {
  if (failsafeFlag) {
    return;
  }

  failsafeFlag = true;
  cruiseActive = false;
  courseHoldActive = false;
  reverseActive = false;
  requestArmState(false, true);
}

void clearRemoteTelemetry() {
  armed = false;
  telemetrySatellites = 0;
  telemetryMainBattery = 0;
  telemetryRemoteBattery = 0;
  linkQuality = 0;
  telemetryRssi = 0;
  telemetryBatteryVoltage = 0.0f;
  telemetrySpeed = 0.0f;
  telemetryHeading = 0.0f;
}

void updateSafetyState(uint32_t now) {
  const bool linkFresh = telemetryIsFresh(now, TELEMETRY_FAILSAFE_MS);

  if (!linkFresh && now > TELEMETRY_FAILSAFE_MS) {
    enterLocalFailsafe(telemetryFailsafeActive);
  } else if (linkFresh) {
    telemetryFailsafeActive = false;
  }

  const bool validMainBattery =
      linkFresh && telemetryBatteryVoltage > 1.0f;
  if (validMainBattery &&
      telemetryBatteryVoltage <= MAIN_BATTERY_DISARM_VOLTAGE) {
    if (mainBatteryLowSinceMs == 0) {
      mainBatteryLowSinceMs = now;
    } else if (now - mainBatteryLowSinceMs >= BATTERY_FAILSAFE_CONFIRM_MS) {
      enterLocalFailsafe(mainBatteryFailsafeActive);
    }
  } else {
    mainBatteryLowSinceMs = 0;
    if (!validMainBattery ||
        telemetryBatteryVoltage > MAIN_BATTERY_DISARM_VOLTAGE + 0.5f) {
      mainBatteryFailsafeActive = false;
    }
  }

  if (TX_BATTERY_MONITOR_ENABLED && batteryVoltage > 2.0f &&
      batteryVoltage <= TX_BATTERY_DISARM_VOLTAGE) {
    if (txBatteryLowSinceMs == 0) {
      txBatteryLowSinceMs = now;
    } else if (now - txBatteryLowSinceMs >= BATTERY_FAILSAFE_CONFIRM_MS) {
      enterLocalFailsafe(txBatteryFailsafeActive);
    }
  } else {
    txBatteryLowSinceMs = 0;
    if (!TX_BATTERY_MONITOR_ENABLED ||
        batteryVoltage > TX_BATTERY_DISARM_VOLTAGE + 0.1f) {
      txBatteryFailsafeActive = false;
    }
  }

  if (lastTelemetryMs != 0 && now - lastTelemetryMs > TELEMETRY_STALE_MS) {
    clearRemoteTelemetry();
  }
}

void updateArmButton() {
  const bool reading = digitalRead(ARM_TOUCH_PIN) == HIGH;
  const uint32_t now = millis();

  if (reading != lastArmTouchReading) {
    lastArmTouchReading = reading;
    lastArmTouchChangeMs = now;
  }

  if (now - lastArmTouchChangeMs < TOUCH_DEBOUNCE_MS) {
    return;
  }

  if (reading == stableArmTouchState) {
    if (stableArmTouchState && !armPressHandled &&
        now - armPressStartedMs >= ARM_HOLD_MS) {
      if (!localFailsafeActive()) {
        requestArmState(true);
      }
      armPressHandled = true;
    }
    return;
  }

  stableArmTouchState = reading;
  if (stableArmTouchState) {
    armPressStartedMs = now;
    armPressHandled = false;

    if (armed || armRequested) {
      requestArmState(false, true);
      armPressHandled = true;
    }
  } else {
    armPressStartedMs = 0;
    armPressHandled = false;
  }
}

void updateCruiseButton() {
  const bool reading = digitalRead(SPEED_HOLD_TOUCH_PIN) == HIGH;
  const uint32_t now = millis();

  if (reading != lastCruiseTouchReading) {
    lastCruiseTouchReading = reading;
    lastCruiseTouchChangeMs = now;
  }

  if (now - lastCruiseTouchChangeMs < TOUCH_DEBOUNCE_MS) {
    return;
  }

  if (reading == stableCruiseTouchState) {
    return;
  }

  stableCruiseTouchState = reading;
  if (stableCruiseTouchState) {
    if (reverseActive && !cruiseActive) {
      return;
    }

    cruiseActive = !cruiseActive;
    if (cruiseActive) {
      cruiseThrottlePercent = motorMappedThrottle(throttlePercent);
    }
  }
}

bool touchPressed(int pin, bool &lastReading, bool &stableState,
                  uint32_t &lastChangeMs) {
  const bool reading = digitalRead(pin) == HIGH;
  const uint32_t now = millis();

  if (reading != lastReading) {
    lastReading = reading;
    lastChangeMs = now;
  }

  if (now - lastChangeMs < TOUCH_DEBOUNCE_MS || reading == stableState) {
    return false;
  }

  stableState = reading;
  return stableState;
}

void updateThrottleNeutralState(uint32_t now) {
  if (throttlePercent == 0) {
    if (throttleNeutralSinceMs == 0) {
      throttleNeutralSinceMs = now;
    }
  } else {
    throttleNeutralSinceMs = 0;
  }
}

void updateCourseHoldButton() {
  if (!touchPressed(COURSE_HOLD_TOUCH_PIN, lastCourseTouchReading,
                    stableCourseTouchState, lastCourseTouchChangeMs)) {
    return;
  }

  if (localFailsafeActive()) {
    return;
  }

  courseHoldActive = !courseHoldActive;
  if (courseHoldActive) {
    courseTargetHeading = telemetryHeading;
  }
}

void updateReverseButton() {
  if (!touchPressed(REVERSE_TOUCH_PIN, lastReverseTouchReading,
                    stableReverseTouchState, lastReverseTouchChangeMs)) {
    return;
  }

  const uint32_t now = millis();
  const bool neutralReady =
      throttleNeutralSinceMs != 0 &&
      now - throttleNeutralSinceMs >= DIRECTION_CHANGE_NEUTRAL_MS;
  if (localFailsafeActive() || !neutralReady) {
    return;
  }

  reverseActive = !reverseActive;
  cruiseActive = false;
  cruiseThrottlePercent = 0;
}

void onTelemetryRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  if (len != sizeof(RemoteTelemetryPacket)) {
    return;
  }

  RemoteTelemetryPacket pkt = {};
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != REMOTE_PACKET_MAGIC || pkt.type != PKT_REMOTE_TELEMETRY) {
    return;
  }

  armed = pkt.armed;
  telemetrySeq = pkt.seq;
  telemetrySatellites = pkt.satellites;
  telemetryMainBattery = pkt.mainBatteryPercent;
  telemetryRemoteBattery = pkt.remoteBatteryPercent;
  linkQuality = pkt.linkQuality;
  telemetryRssi = pkt.rssi;
  telemetryBatteryVoltage = pkt.batteryVoltage;
  telemetrySpeed = pkt.groundSpeed;
  telemetryHeading = pkt.heading;
  lastTelemetryMs = millis();
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  const esp_err_t powerResult = esp_wifi_set_max_tx_power(WIFI_MAX_TX_POWER);

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  esp_now_register_recv_cb(onTelemetryRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RECEIVER_BROADCAST_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  const bool peerOk = esp_now_add_peer(&peer) == ESP_OK;
  serialPrintf("WiFi TX power: %s (requested 19.5 dBm)\n",
               powerResult == ESP_OK ? "OK" : "FAIL");
  return powerResult == ESP_OK && peerOk;
}

void sendControlPacket() {
  RemoteControlPacket pkt = {};
  pkt.magic = REMOTE_PACKET_MAGIC;
  pkt.type = PKT_REMOTE_CONTROL;
  pkt.seq = controlSeq++;
  pkt.txMillis = millis();
  const uint8_t safeThrottle =
      localFailsafeActive() ? 0 : throttleCommandPercent;
  pkt.forwardThrottle = reverseActive ? 0 : safeThrottle;
  pkt.reverseThrottle = reverseActive ? safeThrottle : 0;
  pkt.steering =
      (localFailsafeActive() || courseHoldActive) ? 0 : steeringPercent;
  pkt.armToggleCount = armToggleCount;
  pkt.mode = 0;
  pkt.flags = 0;
  if (imuOk) {
    pkt.flags |= CONTROL_FLAG_IMU_OK;
  }
  if (armRequested) {
    pkt.flags |= CONTROL_FLAG_ARM_REQUESTED;
  }
  if (cruiseActive) {
    pkt.flags |= CONTROL_FLAG_CRUISE_ACTIVE;
  }
  if (courseHoldActive) {
    pkt.flags |= CONTROL_FLAG_COURSE_HOLD_ACTIVE;
  }
  if (reverseActive) {
    pkt.flags |= CONTROL_FLAG_REVERSE_ACTIVE;
  }

  esp_now_send(RECEIVER_BROADCAST_MAC, reinterpret_cast<uint8_t *>(&pkt),
               sizeof(pkt));
}

int16_t px(float angleDeg, int16_t radius) {
  return DISPLAY_CX + static_cast<int16_t>(cosf(angleDeg * DEG_TO_RAD) * radius);
}

int16_t py(float angleDeg, int16_t radius) {
  return DISPLAY_CY + static_cast<int16_t>(sinf(angleDeg * DEG_TO_RAD) * radius);
}

void drawCentered(const char *text, int16_t y, const lgfx::IFont *font,
                  uint16_t color, uint16_t background) {
  canvas.setFont(font);
  canvas.setTextColor(color, background);
  canvas.setCursor(DISPLAY_CX - canvas.textWidth(text) / 2, y);
  canvas.print(text);
}

void drawThickArc(float startDeg, float endDeg, int16_t radius,
                  int16_t thickness, uint16_t color) {
  if (endDeg < startDeg) {
    const float tmp = startDeg;
    startDeg = endDeg;
    endDeg = tmp;
  }

  const int16_t stroke = thickness > 2 ? thickness / 2 : 1;
  const int16_t centerRadius = radius - stroke;

  for (float a = startDeg; a <= endDeg; a += 0.65f) {
    canvas.fillCircle(px(a, centerRadius), py(a, centerRadius), stroke, color);
  }
}

void drawArcTicks(float startDeg, float endDeg, int16_t radius, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    const float t = static_cast<float>(i) / (count - 1);
    const float a = startDeg + (endDeg - startDeg) * t;
    canvas.drawLine(px(a, radius - 7), py(a, radius - 7),
                    px(a, radius - 2), py(a, radius - 2),
                    rgb(54, 76, 86));
  }
}

float angleForPercent(float startDeg, float endDeg, int16_t percent) {
  return startDeg + (endDeg - startDeg) * (percent / 100.0f);
}

uint16_t batteryColor(uint8_t percent) {
  if (percent < 35) {
    return red;
  }
  if (percent < 65) {
    return orange;
  }
  return green;
}

void drawLabel(const char *label, int16_t x, int16_t y) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.setTextColor(textDim, bg);
  canvas.setCursor(x, y);
  canvas.print(label);
}

void drawArcGauge(float startDeg, float endDeg, uint8_t percent,
                  uint16_t color, bool reversed = false) {
  constexpr int arcRadius = 108;
  constexpr int arcThickness = 8;
  constexpr int knobRadius = arcRadius - (arcThickness / 2);
  constexpr int tickRadius = 92;

  drawThickArc(startDeg, endDeg, arcRadius, arcThickness, track);
  drawArcTicks(startDeg, endDeg, tickRadius, 5);

  const float filled = angleForPercent(startDeg, endDeg, percent);
  if (reversed) {
    drawThickArc(filled, endDeg, arcRadius, arcThickness, color);
  } else {
    drawThickArc(startDeg, filled, arcRadius, arcThickness, color);
  }

  canvas.fillCircle(px(filled, knobRadius), py(filled, knobRadius), 4,
                    rgb(230, 238, 245));
}

void drawTurnGauge(int8_t steering) {
  constexpr float start = 222.0f;
  constexpr float end = 318.0f;
  constexpr int arcRadius = 108;
  constexpr int arcThickness = 8;
  constexpr int knobRadius = arcRadius - (arcThickness / 2);
  constexpr int tickRadius = 92;

  const float angle = start + (steering + 100.0f) * 0.5f * (end - start) / 100.0f;

  drawThickArc(start, end, arcRadius, arcThickness, track);
  drawArcTicks(start, end, tickRadius, 7);
  drawThickArc(270, angle, arcRadius, arcThickness, cyan);
  canvas.fillCircle(px(angle, knobRadius), py(angle, knobRadius), 6,
                    rgb(235, 245, 255));
  canvas.drawCircle(px(angle, knobRadius), py(angle, knobRadius), 7,
                    rgb(75, 140, 210));

  char value[8];
  snprintf(value, sizeof(value), "%+d", steering);
  drawCentered(value, 34, &fonts::Font0, rgb(140, 205, 255), bg);
}

float wrapHeadingError(float error) {
  while (error > 180.0f) {
    error -= 360.0f;
  }
  while (error < -180.0f) {
    error += 360.0f;
  }
  return error;
}

void drawHeadingGauge(float headingDeg, bool holdActive, float targetHeading) {
  constexpr float start = 58.0f;
  constexpr float end = 122.0f;
  constexpr float center = 90.0f;
  constexpr int arcRadius = 108;
  constexpr int arcThickness = 8;
  constexpr int knobRadius = arcRadius - (arcThickness / 2);
  constexpr int tickRadius = 92;

  drawThickArc(start, end, arcRadius, arcThickness, track);
  drawArcTicks(start, end, tickRadius, 7);

  float markerAngle = start;
  if (holdActive) {
    const float deviation = constrain(
        wrapHeadingError(headingDeg - targetHeading), -45.0f, 45.0f);
    markerAngle = center + deviation * (end - center) / 45.0f;
    drawThickArc(center, markerAngle, arcRadius, arcThickness, cyan);
  } else {
    const float normalized = fmodf(headingDeg + 360.0f, 360.0f);
    markerAngle = start + normalized * (end - start) / 360.0f;
  }

  canvas.fillCircle(px(markerAngle, knobRadius), py(markerAngle, knobRadius), 5,
                    rgb(235, 245, 255));
  canvas.drawCircle(px(markerAngle, knobRadius), py(markerAngle, knobRadius), 6,
                    rgb(75, 140, 210));

  char value[12];
  snprintf(value, sizeof(value), "%03u deg",
           static_cast<uint16_t>(holdActive ? targetHeading : headingDeg) % 360);
  drawCentered(value, 192, &fonts::Font0, textMain, bg);
  drawCentered(holdActive ? "HLD" : "HDG", 207, &fonts::Font0, textDim, bg);
}

const char *currentModeLabel() {
  if (localFailsafeActive()) {
    return "FAIL";
  }
  if (reverseActive) {
    return "REV";
  }
  if (cruiseActive && courseHoldActive) {
    return "CRZ+HLD";
  }
  if (courseHoldActive) {
    return "HLD";
  }
  if (cruiseActive) {
    return "CRZ";
  }
  return "TILT";
}

void drawCenter(uint8_t satellites, uint16_t speedKmh, uint8_t lq,
                bool linkActive, const char *flightMode, bool isArmed,
                bool blinkOn) {
  canvas.fillCircle(DISPLAY_CX, DISPLAY_CY, 66, panel);
  canvas.drawCircle(DISPLAY_CX, DISPLAY_CY, 66, rgb(55, 78, 86));
  canvas.drawCircle(DISPLAY_CX, DISPLAY_CY, 58, rgb(12, 25, 32));

  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.setTextColor(textDim, panel);
  canvas.setCursor(80, 76);
  canvas.print("MODE");
  canvas.setCursor(135, 76);
  canvas.print("ARM");

  canvas.setTextColor(cyan, panel);
  canvas.setCursor(77, 88);
  canvas.print(flightMode);
  canvas.setTextColor(isArmed ? red : green, panel);
  canvas.setCursor(135, 88);
  canvas.print(isArmed ? "YES" : "NO");

  canvas.drawFastHLine(77, 105, 86, rgb(30, 52, 60));

  const bool lqWarning = linkActive && lq < 90;
  if (!lqWarning || blinkOn) {
    char lqText[12];
    snprintf(lqText, sizeof(lqText), "LQ %u%%", linkActive ? lq : 0);
    drawCentered(lqText, 111, &fonts::Font0,
                 linkActive ? (lqWarning ? red : green) : red, panel);
  }

  char speed[8];
  snprintf(speed, sizeof(speed), "%u", speedKmh);
  drawCentered(speed, 126, &fonts::Font7, textMain, panel);
  drawCentered("km/h", 167, &fonts::Font0, textDim, panel);

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(textDim, panel);
  canvas.setCursor(77, 181);
  canvas.printf("SAT %02u", satellites);
  canvas.setCursor(123, 181);
  if (TX_BATTERY_MONITOR_ENABLED) {
    const bool txWarning = batteryVoltage <= 3.7f;
    if (!txWarning || blinkOn) {
      canvas.setTextColor(txWarning ? red : textDim, panel);
      canvas.printf("TX %.1fV", batteryVoltage);
    }
  } else {
    canvas.print("TX --.-V");
  }
}

void drawDashboard() {
  constexpr float thrStart = 144.0f;
  constexpr float thrEnd = 216.0f;
  constexpr float batStart = 36.0f;
  constexpr float batEnd = -36.0f;

  const uint32_t now = millis();
  const bool blinkOn = (now / 500U) % 2U == 0;
  const bool linkActive = telemetryIsFresh(now, TELEMETRY_STALE_MS);
  const uint8_t mainBattery = linkActive ? telemetryMainBattery : 0;
  const uint8_t satellites = linkActive ? telemetrySatellites : 0;
  const uint16_t speed = linkActive ? static_cast<uint16_t>(telemetrySpeed * 3.6f + 0.5f) : 0;
  const uint16_t heading = linkActive ? static_cast<uint16_t>(telemetryHeading + 0.5f) : 0;
  const float mainVoltage = linkActive ? telemetryBatteryVoltage : 0.0f;
  const char *flightMode = currentModeLabel();
  const uint8_t displayedThrottle = throttleCommandPercent;

  canvas.fillScreen(bg);
  canvas.fillCircle(DISPLAY_CX, DISPLAY_CY, 108, rgb(3, 7, 10));
  canvas.drawCircle(DISPLAY_CX, DISPLAY_CY, 119, rgb(74, 88, 96));
  canvas.drawCircle(DISPLAY_CX, DISPLAY_CY, 111, rgb(20, 32, 38));

  drawTurnGauge(steeringPercent);
  drawArcGauge(thrStart, thrEnd, displayedThrottle, green);
  drawArcGauge(batStart, batEnd, mainBattery, batteryColor(mainBattery));
  drawHeadingGauge(heading, courseHoldActive, courseTargetHeading);

  drawLabel("TURN", 105, 20);
  drawLabel("THR", 30, 112);
  drawLabel("BAT", 184, 112);

  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.setTextColor(textMain, bg);
  canvas.setCursor(34, 130);
  canvas.printf("%u%%", displayedThrottle);
  const bool mainBatteryWarning =
      linkActive && mainVoltage > 1.0f && mainVoltage <= 25.6f;
  if (!mainBatteryWarning || blinkOn) {
    const uint16_t mainBatteryTextColor =
        mainVoltage <= 24.0f ? red : (mainBatteryWarning ? orange : textMain);
    canvas.setTextColor(mainBatteryTextColor, bg);
    canvas.setCursor(188, 130);
    canvas.printf("%u%%", mainBattery);
    canvas.setCursor(188, 143);
    if (linkActive && mainVoltage > 1.0f) {
      canvas.printf("%.1fV", mainVoltage);
    } else {
      canvas.print("--.-V");
    }
  }

  drawCenter(satellites, speed, linkQuality, linkActive, flightMode, armed,
             blinkOn);

  canvas.pushSprite(0, 0);
}

void handleSerialCommands() {
  if (!SERIAL_DEBUG_ENABLED || !Serial) {
    return;
  }

  while (Serial.available()) {
    const char cmd = static_cast<char>(Serial.read());
    if (cmd == 't' || cmd == 'T') {
      hallReleasedRaw = static_cast<uint16_t>(hallFilteredRaw + 0.5f);
      hallMaxRaw = hallReleasedRaw + HALL_MIN_SPAN_RAW;
      hallMaxLearned = false;
      serialPrintf("Throttle rest recalibrat: raw=%u\n", hallReleasedRaw);
    } else if (cmd == 's' || cmd == 'S') {
      calibrateImuRest();
      serialPrintf("Steering rest recalibrat: offset=%.2f deg\n",
                    imuRollOffsetDeg);
    } else if (cmd == 'h' || cmd == 'H' || cmd == '?') {
      serialPrintln("Comenzi:");
      serialPrintln("  t = recalibreaza throttle rest");
      serialPrintln("  s = recalibreaza steering rest");
      serialPrintln("  h = help");
    }
  }
}

void setup() {
  if (SERIAL_DEBUG_ENABLED) {
    Serial.begin(115200);
    delay(100);
  }

  bg = rgb(0, 4, 7);
  panel = rgb(3, 11, 15);
  track = rgb(18, 34, 43);
  textDim = rgb(120, 150, 160);
  textMain = rgb(225, 236, 255);
  cyan = rgb(95, 175, 255);
  green = rgb(75, 222, 140);
  orange = rgb(255, 120, 55);
  yellow = rgb(255, 205, 74);
  red = rgb(255, 85, 75);

  analogReadResolution(12);
  analogSetPinAttenuation(HALL_PIN, ADC_11db);
  if (TX_BATTERY_MONITOR_ENABLED) {
    analogSetPinAttenuation(BATTERY_SENSE_PIN, ADC_11db);
  }
  pinMode(ARM_TOUCH_PIN, INPUT);
  pinMode(SPEED_HOLD_TOUCH_PIN, INPUT);
  pinMode(COURSE_HOLD_TOUCH_PIN, INPUT);
  pinMode(REVERSE_TOUCH_PIN, INPUT);

  if (!initDisplay()) {
    serialPrintln("Display sprite allocation failed.");
    while (true) {
      delay(1000);
    }
  }

  delay(250);
  hallReleasedRaw = calibrateHallReleased();
  hallFilteredRaw = hallReleasedRaw;
  hallMaxRaw = hallReleasedRaw + HALL_MIN_SPAN_RAW;
  hallMaxLearned = false;
  throttleNeutralSinceMs = millis();
  if (TX_BATTERY_MONITOR_ENABLED) {
    updateBatteryVoltage();
  }

  imuOk = initMpu();
  imuRollOffsetDeg = STEERING_MOUNT_OFFSET_DEG;
  const bool espNowOk = initEspNow();

  serialPrintln("Dashboard live: Hall throttle + MPU roll steering Y/Z");
  serialPrintf("Hall OUT -> GPIO%d\n", HALL_PIN);
  serialPrintf("MPU SDA -> GPIO%d, SCL -> GPIO%d, addr 0x%02X\n",
                IMU_SDA_PIN, IMU_SCL_PIN, MPU_ADDR);
  serialPrintf("ARM touch OUT -> GPIO%d\n", ARM_TOUCH_PIN);
  serialPrintf("SPEED HOLD touch OUT -> GPIO%d\n", SPEED_HOLD_TOUCH_PIN);
  serialPrintf("COURSE HOLD touch OUT -> GPIO%d\n", COURSE_HOLD_TOUCH_PIN);
  serialPrintf("REVERSE touch OUT -> GPIO%d\n", REVERSE_TOUCH_PIN);
  serialPrintf("Battery sense -> GPIO%d\n", BATTERY_SENSE_PIN);
  serialPrintf("ESP-NOW: %s\n", espNowOk ? "OK" : "FAIL");
  serialPrintln("Comenzi: t=throttle rest, s=steering rest, h=help");
}

void loop() {
  static uint32_t lastSensor = 0;
  static uint32_t lastFrame = 0;

  const uint32_t now = millis();
  handleSerialCommands();
  updateSafetyState(now);

  if (now - lastSensor >= SENSOR_INTERVAL_MS) {
    lastSensor = now;
    updateThrottle();
    updateThrottleNeutralState(now);
    updateThrottleCommand();
    updateSteering();
    updateArmButton();
    updateCruiseButton();
    updateCourseHoldButton();
    updateReverseButton();
    updateThrottleCommand();
  }

  if (TX_BATTERY_MONITOR_ENABLED &&
      now - lastBatterySampleMs >= BATTERY_SAMPLE_INTERVAL_MS) {
    lastBatterySampleMs = now;
    updateBatteryVoltage();
  }

  if (now - lastControlSendMs >= CONTROL_SEND_INTERVAL_MS) {
    lastControlSendMs = now;
    sendControlPacket();
  }

  if (now - lastFrame >= FRAME_INTERVAL_MS) {
    lastFrame = now;
    drawDashboard();
  }
}
