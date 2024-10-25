#include <bit>
#include <cassert>
#include <iostream>
#include <span>

#include <tmx_cpp/sensors/INA226.hpp>
#include <tmx_cpp/serialization.hpp>

using namespace tmx_cpp;

// struct INA226_MOD_data {
//         float voltage;
//         float current;
// }
INA226_module::INA226_module(uint8_t i2c_port, uint8_t address, INA226_cb_t data_cb)
    : data_cb(data_cb), i2c_port(i2c_port), address(address) {

  this->type = SENSOR_TYPE::INA226;
}

std::vector<uint8_t> INA226_module::init_data() {
  std::cout << "ina port: " << (int)i2c_port << " addr: " << (int)address << std::endl;
  this->type = SENSOR_TYPE::INA226;

  return {i2c_port, address};
}

void INA226_module::data_callback(std::vector<uint8_t> data) {
  static_assert(sizeof(float) == sizeof(uint32_t));
  static_assert(sizeof(float) == 4);
  assert(data.size() == 8);
  auto data_span = std::span(data);

  float voltage_f = decode_float(data_span.first<sizeof(float)>());
  float current_f = decode_float(data_span.subspan<sizeof(float), sizeof(float)>());
  data_cb(voltage_f, current_f);
}
