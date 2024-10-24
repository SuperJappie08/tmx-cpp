#include <cassert>
#include <span>

#include <tmx_cpp/serialization.hpp>
#include <tmx_cpp/sensors/ADXL345.hpp>

using namespace tmx_cpp;

// TODO: Make offset and scaling configurable

ADXL345_module::ADXL345_module(uint8_t i2c_port, uint8_t address, ADXL345_cb_t data_cb)
: data_cb(data_cb), i2c_port(i2c_port), address(address) {
  this->type = SENSOR_TYPE::ADXL345;
}

std::vector<uint8_t> ADXL345_module::init_data() {
  this->type = SENSOR_TYPE::ADXL345;

  // TODO: Address not yet implemented.
  return {i2c_port, /*address*/};
}

void ADXL345_module::data_callback(std::vector<uint8_t> data) {
  const size_t i16_size = sizeof(int16_t);
  assert(data.size() == 3 * i16_size);
  auto data_span = std::span(data);

  // Bytes are swapped in PICO

  // Scaling values are retrieved from SparkFun ADXL 345 Arduino Library
  // LINK: https://github.com/sparkfun/SparkFun_ADXL345_Arduino_Library/blob/6d795cce285d94014c294d297573e33266a4c7d5/src/SparkFun_ADXL345.cpp#L34-L36C15

  // FIXME: Do Something with range somewhere...
  // auto x = 0.00376390f * decode_i16(data_span.subspan<0 * i16_size, i16_size>());
  // auto y = 0.00376009f * decode_i16(data_span.subspan<1 * i16_size, i16_size>());
  // auto z = 0.00349265f * decode_i16(data_span.subspan<2 * i16_size, i16_size>());

  // [THESE VALUES RESULT IN SLIGHTLY BETTER RESULTS]
  // Scaling values are retrieved from no-OS Driver
  // LINK: https://github.com/analogdevicesinc/no-OS/blob/c26d25fe7004edc5a5eef40ca36381b08a187a12/drivers/accel/adxl345/adxl345.h#L183

  auto x = 0.0039f * decode_i16(data_span.subspan<0 * i16_size, i16_size>());
  auto y = 0.0039f * decode_i16(data_span.subspan<1 * i16_size, i16_size>());
  auto z = 0.0039f * decode_i16(data_span.subspan<2 * i16_size, i16_size>());


  data_cb({x, y, z});
}
