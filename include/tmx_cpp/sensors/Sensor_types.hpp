
#pragma once
#include <stdint.h>

namespace tmx_cpp {

enum SENSOR_TYPE : uint8_t {
  ADXL345 = 0x01, // none implemented
  VEML6040 = 0x02,
  INA226 = 0x06,
};

//         GPS = 0
// LOAD_CELL = 1
// MPU_9250 = 2
// TOF_VL53 = 3
// VEML6040 = 4  # Color sensor
// ADXL345 = 5  # // 3 axis accel
// INA226 = 6
// HMC5883 = 7

}