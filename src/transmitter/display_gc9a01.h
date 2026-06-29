#pragma once

#include <LovyanGFX.hpp>

constexpr int TFT_RST = 5;
constexpr int TFT_CS = 4;
constexpr int TFT_DC = 12;
constexpr int TFT_MOSI = 2; // Display pin is usually labeled SDA.
constexpr int TFT_SCLK = 1; // Display pin is usually labeled SCL.

constexpr int DISPLAY_W = 240;
constexpr int DISPLAY_H = 240;
constexpr int DISPLAY_CX = 120;
constexpr int DISPLAY_CY = 120;

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
      cfg.panel_width = DISPLAY_W;
      cfg.panel_height = DISPLAY_H;
      cfg.memory_width = DISPLAY_W;
      cfg.memory_height = DISPLAY_H;
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

extern LGFX display;
extern LGFX_Sprite canvas;

uint16_t rgb(uint8_t r, uint8_t g, uint8_t b);
bool initDisplay();
