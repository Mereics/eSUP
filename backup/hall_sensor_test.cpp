// Backup al testerului Hall KY-035 / 49E.
// Acest fișier NU este compilat de PlatformIO, fiind în afara folderului src/.
// Versiunea activă a proiectului folosește dashboard-ul normal și citește Hall + MPU.

#include <Arduino.h>
#include "display_gc9a01.h"

constexpr int HALL_PIN = 6;

constexpr uint16_t ADC_MAX_VALUE = 4095;
constexpr float ADC_REF_VOLTAGE = 3.3f;
constexpr uint16_t SAMPLE_INTERVAL_MS = 20;
constexpr uint16_t DISPLAY_INTERVAL_MS = 80;

uint16_t bg;
uint16_t panel;
uint16_t track;
uint16_t textDim;
uint16_t textMain;
uint16_t cyan;
uint16_t green;
uint16_t orange;

float filteredRaw = 0.0f;
uint16_t rawMin = ADC_MAX_VALUE;
uint16_t rawMax = 0;
uint16_t releasedRaw = 0;
uint16_t fullRaw = 0;
bool releasedCaptured = false;
bool fullCaptured = false;

uint16_t readHallRaw() {
  uint32_t sum = 0;
  constexpr uint8_t samples = 16;

  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(HALL_PIN);
    delayMicroseconds(250);
  }

  return sum / samples;
}

float rawToVoltage(float raw) {
  return raw * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
}

uint8_t travelPercent(float raw) {
  if (!releasedCaptured || !fullCaptured || releasedRaw == fullRaw) {
    return 0;
  }

  const float travel = (raw - releasedRaw) * 100.0f / (fullRaw - releasedRaw);
  return static_cast<uint8_t>(constrain(travel, 0.0f, 100.0f));
}

void drawCentered(const char *text, int16_t y, const lgfx::IFont *font,
                  uint16_t color, uint16_t background) {
  canvas.setFont(font);
  canvas.setTextColor(color, background);
  canvas.setCursor(DISPLAY_CX - canvas.textWidth(text) / 2, y);
  canvas.print(text);
}

void drawTravelBar(int16_t x, int16_t y, int16_t w, int16_t h,
                   uint8_t percent) {
  canvas.fillRoundRect(x, y, w, h, 6, track);
  canvas.drawRoundRect(x, y, w, h, 6, rgb(80, 110, 125));

  const int16_t fillW = map(percent, 0, 100, 0, w - 8);
  canvas.fillRoundRect(x + 4, y + 4, fillW, h - 8, 4, green);
}

void drawHallScreen(uint16_t raw, float filtered, uint16_t minValue,
                    uint16_t maxValue, uint16_t releasedValue,
                    uint16_t fullValue, uint8_t travel, bool calibrated) {
  canvas.fillScreen(bg);
  canvas.fillCircle(DISPLAY_CX, DISPLAY_CY, 112, rgb(3, 8, 12));
  canvas.drawCircle(DISPLAY_CX, DISPLAY_CY, 119, rgb(74, 88, 96));
  canvas.drawCircle(DISPLAY_CX, DISPLAY_CY, 106, rgb(22, 38, 48));

  drawCentered("49E HALL TEST", 17, &fonts::Font2, textMain, bg);

  char mainValue[12];
  snprintf(mainValue, sizeof(mainValue), "%u%%", travel);
  drawCentered(mainValue, 98, &fonts::Font7, textMain, rgb(3, 8, 12));
  drawCentered(calibrated ? "trigger travel" : "send f for full",
               145, &fonts::Font0, textDim, rgb(3, 8, 12));

  drawTravelBar(34, 166, 172, 16, travel);

  canvas.fillRoundRect(35, 188, 170, 34, 8, panel);
  canvas.drawRoundRect(35, 188, 170, 34, 8, rgb(42, 70, 82));

  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.setTextColor(textDim, panel);
  canvas.setCursor(46, 195);
  canvas.print("RAW");
  canvas.setCursor(100, 195);
  canvas.print("VOLT");
  canvas.setCursor(154, 195);
  canvas.print("TRVL");

  canvas.setTextColor(cyan, panel);
  canvas.setCursor(46, 207);
  canvas.printf("%u", raw);
  canvas.setCursor(100, 207);
  canvas.printf("%.2f", rawToVoltage(filtered));
  canvas.setCursor(154, 207);
  canvas.printf("%u%%", travel);

  canvas.setTextColor(textDim, bg);
  canvas.setCursor(26, 50);
  canvas.printf("MIN %u", minValue);
  canvas.setCursor(159, 50);
  canvas.printf("MAX %u", maxValue);
  canvas.setCursor(22, 226);
  canvas.printf("REL %u", releasedValue);
  canvas.setCursor(139, 226);
  canvas.printf("FULL %u", fullValue);

  canvas.pushSprite(0, 0);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  bg = rgb(0, 4, 7);
  panel = rgb(3, 11, 15);
  track = rgb(18, 34, 43);
  textDim = rgb(120, 150, 160);
  textMain = rgb(225, 236, 255);
  cyan = rgb(95, 175, 255);
  green = rgb(75, 222, 140);
  orange = rgb(255, 120, 55);

  analogReadResolution(12);
  analogSetPinAttenuation(HALL_PIN, ADC_11db);

  if (!initDisplay()) {
    while (true) {
      delay(1000);
    }
  }

  delay(250);
  releasedRaw = readHallRaw();
  filteredRaw = releasedRaw;
  rawMin = releasedRaw;
  rawMax = releasedRaw;
  releasedCaptured = true;
}

void loop() {
  static uint32_t lastSample = 0;
  static uint32_t lastDisplay = 0;

  const uint32_t now = millis();

  while (Serial.available()) {
    const char cmd = static_cast<char>(Serial.read());
    const uint16_t current = static_cast<uint16_t>(filteredRaw + 0.5f);
    if (cmd == 'f' || cmd == 'F') {
      fullRaw = current;
      fullCaptured = true;
    }
  }

  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;
    const uint16_t raw = readHallRaw();
    filteredRaw = filteredRaw * 0.86f + raw * 0.14f;
    rawMin = min<uint16_t>(rawMin, raw);
    rawMax = max<uint16_t>(rawMax, raw);
  }

  if (now - lastDisplay >= DISPLAY_INTERVAL_MS) {
    lastDisplay = now;
    const uint16_t filtered = static_cast<uint16_t>(filteredRaw + 0.5f);
    drawHallScreen(filtered, filteredRaw, rawMin, rawMax, releasedRaw, fullRaw,
                   travelPercent(filteredRaw), fullCaptured);
  }
}
