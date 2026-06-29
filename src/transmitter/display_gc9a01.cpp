#include "display_gc9a01.h"

LGFX display;
LGFX_Sprite canvas(&display);

uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return display.color565(r, g, b);
}

bool initDisplay() {
  display.init();
  display.setRotation(0);
  display.setBrightness(255);

  canvas.setColorDepth(16);
  return canvas.createSprite(DISPLAY_W, DISPLAY_H);
}
