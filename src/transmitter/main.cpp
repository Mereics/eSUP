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
constexpr int ARM_TOUCH_PIN = 9;
constexpr int CRUISE_TOUCH_PIN = 10;
constexpr int BATTERY_SENSE_PIN = 11;
constexpr uint8_t MPU_ADDR = 0x68;

constexpr uint16_t ADC_MAX_VALUE = 4095;
constexpr float ADC_REF_VOLTAGE = 3.3f;

constexpr uint16_t FRAME_INTERVAL_MS = 80;
constexpr uint16_t SENSOR_INTERVAL_MS = 15;
constexpr uint16_t HALL_BOOT_CALIBRATION_MS = 700;
constexpr uint16_t HALL_MIN_SPAN_RAW = 700;
constexpr uint16_t HALL_NOISE_DEADBAND_RAW = 25;
constexpr uint16_t HALL_START_LEARN_THRESHOLD_RAW = 80;
constexpr uint8_t THROTTLE_INPUT_DEADBAND_PERCENT = 2;
constexpr uint8_t MOTOR_START_PERCENT = 15;
constexpr float BATTERY_DIVIDER_RATIO = 2.0f;
constexpr uint16_t BATTERY_SAMPLE_INTERVAL_MS = 250;
constexpr bool SERIAL_DEBUG_ENABLED = false;

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
constexpr uint16_t TOUCH_DEBOUNCE_MS = 60;

bool cruiseActive = false;
uint8_t cruiseThrottlePercent = 0;
bool lastCruiseTouchReading = false;
bool stableCruiseTouchState = false;
uint32_t lastCruiseTouchChangeMs = 0;

float batteryVoltage = 0.0f;
uint8_t transmitterBatteryPercent = 100;
uint32_t lastBatterySampleMs = 0;

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
  if (inputPercent <= THROTTLE_INPUT_DEADBAND_PERCENT) {
    return 0;
  }

  return static_cast<uint8_t>(
      map(inputPercent, THROTTLE_INPUT_DEADBAND_PERCENT, 100,
          MOTOR_START_PERCENT, 100));
}

void updateThrottleCommand() {
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
    return;
  }

  stableArmTouchState = reading;
  if (stableArmTouchState) {
    armRequested = !armRequested;
    armToggleCount++;
  }
}

void updateCruiseButton() {
  const bool reading = digitalRead(CRUISE_TOUCH_PIN) == HIGH;
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
    cruiseActive = !cruiseActive;
    if (cruiseActive) {
      cruiseThrottlePercent = motorMappedThrottle(throttlePercent);
    }
  }
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

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  esp_now_register_recv_cb(onTelemetryRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RECEIVER_BROADCAST_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void sendControlPacket() {
  RemoteControlPacket pkt = {};
  pkt.magic = REMOTE_PACKET_MAGIC;
  pkt.type = PKT_REMOTE_CONTROL;
  pkt.seq = controlSeq++;
  pkt.txMillis = millis();
  pkt.throttle = throttleCommandPercent;
  pkt.steering = steeringPercent;
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

void drawCenter(uint8_t satellites, uint8_t remoteBattery, uint16_t speedKmh,
                const char *flightMode, bool armed, uint16_t headingDeg) {
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
  canvas.setCursor(80, 88);
  canvas.print(flightMode);
  canvas.setTextColor(armed ? red : green, panel);
  canvas.setCursor(135, 88);
  canvas.print(armed ? "YES" : "NO");

  canvas.drawFastHLine(77, 105, 86, rgb(30, 52, 60));

  char heading[8];
  snprintf(heading, sizeof(heading), "%03u", headingDeg % 360);
  canvas.setTextColor(textDim, panel);
  canvas.setCursor(82, 111);
  canvas.print("HDG");
  canvas.setTextColor(yellow, panel);
  canvas.setCursor(112, 111);
  canvas.print(heading);
  canvas.setTextColor(textDim, panel);
  canvas.setCursor(136, 111);
  canvas.print("deg");

  char speed[8];
  snprintf(speed, sizeof(speed), "%u", speedKmh);
  drawCentered(speed, 130, &fonts::Font7, textMain, panel);
  drawCentered("km/h", 171, &fonts::Font0, textDim, panel);

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(textDim, panel);
  canvas.setCursor(79, 181);
  canvas.printf("SAT %02u", satellites);
  canvas.setCursor(125, 181);
  canvas.printf("TX %02u%%", remoteBattery);
}

void drawDashboard() {
  constexpr float thrStart = 144.0f;
  constexpr float thrEnd = 216.0f;
  constexpr float pwrStart = 36.0f;
  constexpr float pwrEnd = -36.0f;
  constexpr float batStart = 58.0f;
  constexpr float batEnd = 122.0f;

  const uint8_t powerPercent = throttleCommandPercent;
  const bool linkActive = millis() - lastTelemetryMs < 1000;
  const uint8_t mainBattery = linkActive ? telemetryMainBattery : 0;
  const uint8_t remoteBattery = transmitterBatteryPercent;
  const uint8_t satellites = linkActive ? telemetrySatellites : 0;
  const uint16_t speed = linkActive ? static_cast<uint16_t>(telemetrySpeed * 3.6f + 0.5f) : 0;
  const uint16_t heading = linkActive ? static_cast<uint16_t>(telemetryHeading + 0.5f) : 0;
  const char *flightMode = cruiseActive ? "CRZ" : "MAN";

  canvas.fillScreen(bg);
  canvas.fillCircle(DISPLAY_CX, DISPLAY_CY, 108, rgb(3, 7, 10));
  canvas.drawCircle(DISPLAY_CX, DISPLAY_CY, 119, rgb(74, 88, 96));
  canvas.drawCircle(DISPLAY_CX, DISPLAY_CY, 111, rgb(20, 32, 38));

  drawTurnGauge(steeringPercent);
  drawArcGauge(thrStart, thrEnd, throttleCommandPercent, green);
  drawArcGauge(pwrStart, pwrEnd, powerPercent, orange);
  drawArcGauge(batStart, batEnd, mainBattery, batteryColor(mainBattery), true);

  drawLabel("TURN", 105, 20);
  drawLabel("THR", 30, 112);
  drawLabel("PWR", 184, 112);
  drawLabel("BAT", 105, 207);

  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.setTextColor(textMain, bg);
  canvas.setCursor(34, 130);
  canvas.printf("%u%%", throttleCommandPercent);
  canvas.setCursor(184, 130);
  canvas.printf("%u%%", powerPercent);

  char mainBatteryText[8];
  snprintf(mainBatteryText, sizeof(mainBatteryText), "%u%%", mainBattery);
  drawCentered(mainBatteryText, 192, &fonts::Font0, textMain, bg);

  drawCenter(satellites, remoteBattery, speed, flightMode, armed, heading);

  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.setTextColor(linkActive ? green : red, bg);
  canvas.setCursor(93, 52);
  canvas.printf("LQ %u%%", linkActive ? linkQuality : 0);

  canvas.setTextColor(textDim, bg);
  canvas.setCursor(80, 226);
  canvas.printf("TX %.2fV", batteryVoltage);

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
  analogSetPinAttenuation(BATTERY_SENSE_PIN, ADC_11db);
  pinMode(ARM_TOUCH_PIN, INPUT);
  pinMode(CRUISE_TOUCH_PIN, INPUT);

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
  updateBatteryVoltage();

  imuOk = initMpu();
  imuRollOffsetDeg = STEERING_MOUNT_OFFSET_DEG;
  const bool espNowOk = initEspNow();

  serialPrintln("Dashboard live: Hall throttle + MPU roll steering Y/Z");
  serialPrintf("Hall OUT -> GPIO%d\n", HALL_PIN);
  serialPrintf("MPU SDA -> GPIO%d, SCL -> GPIO%d, addr 0x%02X\n",
                IMU_SDA_PIN, IMU_SCL_PIN, MPU_ADDR);
  serialPrintf("ARM touch OUT -> GPIO%d\n", ARM_TOUCH_PIN);
  serialPrintf("CRUISE touch OUT -> GPIO%d\n", CRUISE_TOUCH_PIN);
  serialPrintf("Battery sense -> GPIO%d\n", BATTERY_SENSE_PIN);
  serialPrintf("ESP-NOW: %s\n", espNowOk ? "OK" : "FAIL");
  serialPrintln("Comenzi: t=throttle rest, s=steering rest, h=help");
}

void loop() {
  static uint32_t lastSensor = 0;
  static uint32_t lastFrame = 0;

  const uint32_t now = millis();
  handleSerialCommands();

  if (now - lastSensor >= SENSOR_INTERVAL_MS) {
    lastSensor = now;
    updateThrottle();
    updateThrottleCommand();
    updateSteering();
    updateArmButton();
    updateCruiseButton();
    updateThrottleCommand();
  }

  if (now - lastBatterySampleMs >= BATTERY_SAMPLE_INTERVAL_MS) {
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
