// Backup al dashboard-ului GC9A01 făcut înainte de testul Hall.
// Acest fișier NU este compilat de PlatformIO, fiind în afara folderului src/.
// Când vrem să revenim la UI, putem copia logica de aici în src/main.cpp sau
// o putem transforma într-un modul DashboardUI.

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <SPI.h>

constexpr int TFT_RST = 5;
constexpr int TFT_CS = 4;
constexpr int TFT_DC = 3;
constexpr int TFT_MOSI = 2;
constexpr int TFT_SCLK = 1;

constexpr int W = 240;
constexpr int H = 240;
constexpr int CX = 120;
constexpr int CY = 120;

constexpr float TURN_START = 222.0f;
constexpr float TURN_END = 318.0f;
constexpr float THR_START = 144.0f;
constexpr float THR_END = 216.0f;
constexpr float PWR_START = -36.0f;
constexpr float PWR_END = 36.0f;
constexpr float BAT_START = 58.0f;
constexpr float BAT_END = 122.0f;

constexpr int OUTER_ARC_RADIUS = 108;
constexpr int OUTER_ARC_THICKNESS = 8;
constexpr int KNOB_RADIUS = OUTER_ARC_RADIUS - (OUTER_ARC_THICKNESS / 2);
constexpr int TICK_RADIUS = 92;

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI _bus;

public:
  LGFX() {
    {
      auto cfg = _bus.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = false;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = TFT_SCLK;
      cfg.pin_mosi = TFT_MOSI;
      cfg.pin_miso = -1;
      cfg.pin_dc = TFT_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }

    {
      auto cfg = _panel.config();
      cfg.pin_cs = TFT_CS;
      cfg.pin_rst = TFT_RST;
      cfg.pin_busy = -1;
      cfg.panel_width = W;
      cfg.panel_height = H;
      cfg.memory_width = W;
      cfg.memory_height = H;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = false;
      cfg.invert = true;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel.config(cfg);
    }

    setPanel(&_panel);
  }
};

LGFX display;
LGFX_Sprite canvas(&display);

uint16_t bg;
uint16_t panel;
uint16_t track;
uint16_t textDim;
uint16_t textMain;
uint16_t cyan;
uint16_t green;
uint16_t orange;
uint16_t yellow;

uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return display.color565(r, g, b);
}

int16_t px(float angleDeg, int16_t radius) {
  return CX + static_cast<int16_t>(cosf(angleDeg * DEG_TO_RAD) * radius);
}

int16_t py(float angleDeg, int16_t radius) {
  return CY + static_cast<int16_t>(sinf(angleDeg * DEG_TO_RAD) * radius);
}

void drawCentered(const char *text, int16_t y, const lgfx::IFont *font,
                  uint16_t color, uint16_t background) {
  canvas.setFont(font);
  canvas.setTextColor(color, background);
  canvas.setCursor(CX - canvas.textWidth(text) / 2, y);
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
  const float step = radius > 100 ? 0.65f : 1.0f;

  for (float a = startDeg; a <= endDeg; a += step) {
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

float angleForPercent(float startDeg, float endDeg, uint8_t percent) {
  return startDeg + (endDeg - startDeg) * (percent / 100.0f);
}

uint16_t batteryColor(uint8_t percent) {
  if (percent < 35) {
    return rgb(245, 70, 55);
  }
  if (percent < 65) {
    return rgb(240, 170, 55);
  }
  return rgb(70, 220, 120);
}

void drawLabel(const char *label, int16_t x, int16_t y) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.setTextColor(textDim, bg);
  canvas.setCursor(x, y);
  canvas.print(label);
}

void drawArcGauge(float startDeg, float endDeg, uint8_t percent,
                  uint16_t color, bool reversed = false,
                  bool knob = true) {
  drawThickArc(startDeg, endDeg, OUTER_ARC_RADIUS, OUTER_ARC_THICKNESS, track);
  drawArcTicks(startDeg, endDeg, TICK_RADIUS, 5);

  const float filled = angleForPercent(startDeg, endDeg, percent);
  if (reversed) {
    drawThickArc(filled, endDeg, OUTER_ARC_RADIUS, OUTER_ARC_THICKNESS, color);
  } else {
    drawThickArc(startDeg, filled, OUTER_ARC_RADIUS, OUTER_ARC_THICKNESS, color);
  }

  if (knob) {
    canvas.fillCircle(px(filled, KNOB_RADIUS), py(filled, KNOB_RADIUS), 4,
                      rgb(230, 238, 245));
  }
}

void drawTurn(float turn) {
  const float start = TURN_START;
  const float end = TURN_END;
  const float angle = start + (turn + 1.0f) * 0.5f * (end - start);

  drawThickArc(start, end, OUTER_ARC_RADIUS, OUTER_ARC_THICKNESS, track);
  drawArcTicks(start, end, TICK_RADIUS, 7);
  drawThickArc(270, angle, OUTER_ARC_RADIUS, OUTER_ARC_THICKNESS, cyan);
  canvas.fillCircle(px(angle, KNOB_RADIUS), py(angle, KNOB_RADIUS), 6,
                    rgb(235, 245, 255));
  canvas.drawCircle(px(angle, KNOB_RADIUS), py(angle, KNOB_RADIUS), 7,
                    rgb(75, 140, 210));

  char value[8];
  snprintf(value, sizeof(value), "%+d", static_cast<int>(turn * 45.0f));
  drawCentered(value, 34, &fonts::Font0, rgb(140, 205, 255), bg);
}

void drawCenter(uint8_t satellites, uint8_t remoteBattery, uint16_t speedKmh,
                const char *flightMode, bool armed, uint16_t headingDeg) {
  canvas.fillCircle(CX, CY, 66, panel);
  canvas.drawCircle(CX, CY, 66, rgb(55, 78, 86));
  canvas.drawCircle(CX, CY, 58, rgb(12, 25, 32));

  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.setTextColor(textDim, panel);
  canvas.setCursor(80, 76);
  canvas.print("MODE");
  canvas.setCursor(135, 76);
  canvas.print("ARM");

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(cyan, panel);
  canvas.setCursor(80, 88);
  canvas.print(flightMode);
  canvas.setTextColor(armed ? rgb(255, 85, 75) : rgb(80, 220, 140), panel);
  canvas.setCursor(135, 88);
  canvas.print(armed ? "YES" : "NO");

  canvas.drawFastHLine(77, 105, 86, rgb(30, 52, 60));

  char heading[8];
  snprintf(heading, sizeof(heading), "%03u", headingDeg % 360);
  canvas.setFont(&fonts::Font0);
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

void drawDashboard(float turn, uint8_t throttle, uint8_t power,
                   uint8_t mainBattery, uint8_t remoteBattery,
                   uint8_t satellites, uint16_t speedKmh,
                   const char *flightMode, bool armed,
                   uint16_t headingDeg) {
  canvas.fillScreen(bg);

  canvas.fillCircle(CX, CY, 108, rgb(3, 7, 10));
  canvas.drawCircle(CX, CY, 119, rgb(74, 88, 96));
  canvas.drawCircle(CX, CY, 111, rgb(20, 32, 38));

  drawTurn(turn);
  drawArcGauge(THR_START, THR_END, throttle, green);
  drawArcGauge(PWR_START, PWR_END, power, orange, true);
  drawArcGauge(BAT_START, BAT_END, mainBattery, batteryColor(mainBattery), true);

  drawLabel("TURN", 105, 20);
  drawLabel("THR", 30, 112);
  drawLabel("PWR", 184, 112);
  drawLabel("BAT", 105, 207);

  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.setTextColor(textMain, bg);
  canvas.setCursor(34, 130);
  canvas.printf("%u%%", throttle);
  canvas.setCursor(184, 130);
  canvas.printf("%u%%", power);

  char mainBat[8];
  snprintf(mainBat, sizeof(mainBat), "%u%%", mainBattery);
  drawCentered(mainBat, 192, &fonts::Font0, textMain, bg);

  drawCenter(satellites, remoteBattery, speedKmh, flightMode, armed,
             headingDeg);
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
  yellow = rgb(255, 205, 74);

  display.init();
  display.setRotation(0);
  display.setBrightness(255);

  canvas.setColorDepth(16);
  if (!canvas.createSprite(W, H)) {
    Serial.println("Failed to allocate 240x240 sprite.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  static uint32_t lastFrame = 0;
  if (millis() - lastFrame < 80) {
    return;
  }
  lastFrame = millis();

  const float t = millis() / 1000.0f;
  const float turn = sinf(t * 0.9f);
  const uint8_t throttle = 48 + static_cast<int8_t>(sinf(t * 0.55f) * 42.0f);
  const uint8_t power = 52 + static_cast<int8_t>(cosf(t * 0.65f) * 38.0f);
  const uint8_t mainBattery = 74 + static_cast<int8_t>(sinf(t * 0.16f) * 10.0f);
  const uint8_t remoteBattery = 91 + static_cast<int8_t>(cosf(t * 0.22f) * 6.0f);
  const uint8_t satellites = 8 + static_cast<uint8_t>((sinf(t * 0.38f) + 1.0f) * 2.0f);
  const uint16_t speed = 36 + static_cast<uint16_t>((sinf(t * 0.48f) + 1.0f) * 27.0f);
  const char *flightMode = static_cast<int>(t / 5.0f) % 2 == 0 ? "MAN" : "SPT";
  const bool armed = static_cast<int>(t / 7.0f) % 2 == 0;
  const uint16_t heading = static_cast<uint16_t>(fmodf(t * 34.0f, 360.0f));

  drawDashboard(turn, throttle, power, mainBattery, remoteBattery, satellites,
                speed, flightMode, armed, heading);
}
