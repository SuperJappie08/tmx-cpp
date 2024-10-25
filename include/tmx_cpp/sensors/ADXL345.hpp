#pragma once
#include <array>

#include <tmx_cpp/sensors/Sensor_t.hpp>

namespace tmx_cpp {

// TODO: Add calibration support (There are registers for this)

using ADXL345_cb_t = std::function<void(std::array<float, 3> accelaration)>;
class ADXL345_module : public Sensor_type {
public:
  std::function<void(std::vector<uint8_t>)> send_module;
  ADXL345_module(uint8_t i2c_port, uint8_t address, ADXL345_cb_t data_cb);
  std::vector<uint8_t> init_data() override;

  virtual void data_callback(std::vector<uint8_t> data) override;
  ADXL345_cb_t data_cb;

private:
  uint8_t i2c_port = 0;
  // I2C address can be 0x53 or 0x1D (When SDO is pulled high)
  uint8_t address = 0x53;
};

}  // namespace tmx_cpp