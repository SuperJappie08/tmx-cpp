
#pragma once
#include <stdint.h>
enum MODULE_TYPE : uint8_t {
  PCA9685 = 0,
  HIWONDER_SERVO = 1,
  SHUTDOWN_RELAY = 2, // Not implemented, TODO
  TMX_SSD1306 = 3,
};
