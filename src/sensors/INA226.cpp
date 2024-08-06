#include <bit>
#include <sensors/INA226.hpp>
// struct INA226_MOD_data {
//         float voltage;
//         float current;
// }
INA226_module::INA226_module(uint8_t i2c_port, uint8_t address,
                             INA226_cb_t data_cb)
    : i2c_port(i2c_port), address(address), data_cb(data_cb) {}

std::vector<uint8_t> INA226_module::init_data() { return {i2c_port, address}; }

void INA226_module::data_callback(std::vector<uint8_t> data) {
  static_assert(sizeof(float) == sizeof(uint32_t));
  static_assert(sizeof(float) == 4);
  uint32_t temp = (((uint32_t)data[0]) << 24) | (((uint32_t)data[1]) << 16) |
                  (((uint32_t)data[2]) << 8) | ((uint32_t)data[3]);
  float voltage_f = std::bit_cast<float>(temp);
  temp = (((uint32_t)data[4]) << 24) | (((uint32_t)data[5]) << 16) |
         (((uint32_t)data[6]) << 8) | ((uint32_t)data[7]);
  float current_f = std::bit_cast<float>(temp);
  data_cb(voltage_f, current_f);
}
