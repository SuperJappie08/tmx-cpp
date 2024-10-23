#include <cassert>
#include <cstdint>

#include <tmx_cpp/sensors/VEML6040.hpp>
#include <tmx_cpp/serialization.hpp>

using namespace tmx_cpp;

VEML6040_module::VEML6040_module(uint8_t i2c_port, uint8_t address, VEML6040_cb_t data_cb)
: data_cb(data_cb), i2c_port(i2c_port), address(address) {
  assert(this->address == 0x10);
  this->type = SENSOR_TYPE::VEML6040;
}

std::vector<uint8_t> VEML6040_module::init_data() {
  this->type = SENSOR_TYPE::VEML6040;

  // Address is fixed in PICO Firmware and on the Hardware.
  return {i2c_port, /*address*/};
}

void VEML6040_module::data_callback(std::vector<uint8_t> data) {
  // Get the data from the VEML6040 sensor
  assert(data.size() == 8);
  auto data_span = std::span(data);

  uint16_t red = decode_u16(data_span.first<sizeof(uint16_t)>());
  uint16_t green = decode_u16(data_span.subspan<sizeof(uint16_t), sizeof(uint16_t)>());
  uint16_t blue = decode_u16(data_span.subspan<(sizeof(uint16_t) * 2), sizeof(uint16_t)>());
  uint16_t white = decode_u16(data_span.subspan<(sizeof(uint16_t) * 3), sizeof(uint16_t)>());

  data_cb(red, green, blue, white);
}
