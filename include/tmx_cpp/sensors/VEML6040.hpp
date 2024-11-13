#pragma once
#include "tmx_cpp/sensors/Sensor_t.hpp"
#include <functional>

namespace tmx_cpp {

using VEML6040_cb_t =
    std::function<void(uint16_t red, uint16_t green, uint16_t blue, uint16_t white)>;
class VEML6040_module : public Sensor_type {
public:
  std::function<void(std::vector<uint8_t>)> send_module;
  VEML6040_module(uint8_t i2c_port, uint8_t address, VEML6040_cb_t data_cb);

  virtual std::vector<uint8_t> init_data() override;
  virtual void data_callback(std::vector<uint8_t> data) override;
  VEML6040_cb_t data_cb;

private:
  uint8_t i2c_port = 0;
  uint8_t address = 0x10;
};

} // namespace tmx_cpp
