#include <cstdint>
#include <cassert>

#include <tmx_cpp/sensors/VEML6040.hpp>

using namespace tmx_cpp;

VEML6040_module::VEML6040_module(uint8_t i2c_port, uint8_t address, VEML6040_cb_t data_cb):
  data_cb(data_cb), i2c_port(i2c_port), address(address) {
    assert(this->address == 0x10);
    this->type = SENSOR_TYPE::VEML6040;
  }

std::vector<uint8_t> VEML6040_module::init_data() {
  this->type = SENSOR_TYPE::VEML6040;

  // Address is fixed in PICO Firmware and on the Hardware.
  return {i2c_port , /*address*/};
}

void VEML6040_module::data_callback(std::vector<uint8_t> data) {
  // Get the data from the VEML6040 sensor
  assert(data.size() == 8);
  uint16_t red = ((uint16_t)data[1] << 8) | ((uint16_t)data[0]);
  uint16_t green = ((uint16_t)data[3] << 8) | ((uint16_t)data[2]);
  uint16_t blue = ((uint16_t)data[5] << 8) | ((uint16_t)data[4]);
  uint16_t white = ((uint16_t)data[7] << 8) | ((uint16_t)data[6]);

  data_cb(red, green, blue, white);
}
