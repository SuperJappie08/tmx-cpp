
#pragma once
#include <stdint.h>

namespace tmx_cpp {

enum SENSOR_TYPE : uint8_t {
  MPU9250 = 0x02,
  VEML6040 = 0x04, // TODO: Not implemented
  ADXL345 = 0x05,  // // TODO:  Not implemented
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

} // namespace tmx_cpp