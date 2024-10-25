#pragma once
#include <stdint.h>
#include <array>
#include <functional>
#include "tmx_cpp/sensors/Sensor_t.hpp"

namespace tmx_cpp {

using MPU9250_cb_t = std::function<void(
  std::array<float, 3> acceleration, std::array<float, 3> gyro, std::array<float, 3> magnetic_field,
  std::array<float, 4> quaternion)>;
class MPU9250_module : public Sensor_type {
public:
  std::function<void(std::vector<uint8_t>)> send_module;
  MPU9250_module(uint8_t i2c_port, uint8_t address, MPU9250_cb_t data_cb);

  std::vector<uint8_t> init_data() override;
  virtual void data_callback(std::vector<uint8_t> data) override;
  MPU9250_cb_t data_cb;

private:
  uint8_t i2c_port = 0;
  uint8_t address = 0x68;
};

}  // namespace tmx_cpp
