#pragma once
#include "tmx_cpp/sensors/Sensor_types.hpp"
#include <functional>
#include <stdint.h>
#include <vector>

namespace tmx_cpp {

class Sensor_type {
public:
  virtual std::vector<uint8_t> init_data() = 0;
  virtual void data_callback(std::vector<uint8_t> data) = 0;
  SENSOR_TYPE type;
  virtual ~Sensor_type() = default;
};

}