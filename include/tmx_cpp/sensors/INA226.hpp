#pragma once
#include "tmx_cpp/sensors/Sensor_t.hpp"
#include <functional>
#include <stdint.h>
#include <vector>

namespace tmx_cpp {

using INA226_cb_t = std::function<void(float voltage, float current)>;
class INA226_module : public Sensor_type {
public:
  std::function<void(std::vector<uint8_t>)> send_module;
  INA226_module(uint8_t i2c_port, uint8_t address, INA226_cb_t data_cb);

  std::vector<uint8_t> init_data();
  void data_callback(std::vector<uint8_t> data);
  INA226_cb_t data_cb;

private:
  uint8_t i2c_port = 0;
  uint8_t address = 0x40;
  // time switch_trigger_start_time;
};

// min volt
// max volt
// max curr
// power low time
// port, id

}