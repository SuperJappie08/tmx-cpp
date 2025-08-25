#define _USE_MATH_DEFINES

#include <algorithm>
#include <bit>
#include <cassert>
#include <cmath>
#include <iostream>
#include <span>

#include <tmx_cpp/sensors/AS5600.hpp>
#include <tmx_cpp/serialization.hpp>

using namespace tmx_cpp;

// struct AS5600_MOD_data {
//         float voltage;
//         float current;
// }
AS5600_tmx_sensor::AS5600_tmx_sensor(uint8_t i2c_port, uint8_t address,
                                     std::vector<int> channels,
                                     AS5600_cb_t data_cb)
    : data_cb(data_cb), i2c_port(i2c_port), channels(channels) {
  // sort channels to ensure consistent order
  std::sort(this->channels.begin(), this->channels.end());
  // this->address = address;
  this->mux = 0x00; // Default mux value
  for (const auto &channel : channels) {
    if (channel < 0 || channel > 7) {
      throw std::invalid_argument("Channel must be between 0 and 7");
    }
    mux |= (1 << channel); // Set the corresponding bit for each channel
  }

  // Set the sensor type
  this->type = SENSOR_TYPE::AS5600;
}

std::vector<uint8_t> AS5600_tmx_sensor::init_data() {
  // std::cout << "as5600 port: " << (int)i2c_port << " addr:
  this->type = SENSOR_TYPE::AS5600;

  return {i2c_port, this->mux};
}

void AS5600_tmx_sensor::data_callback(std::vector<uint8_t> data) {
  static_assert(sizeof(float) == sizeof(uint32_t));
  static_assert(sizeof(float) == 4);
  assert(data.size() == channels.size() * 2);
  auto data_span = std::span(data);
  std::vector<std::tuple<int, float, int>> results;
  for (size_t i = 0; i < channels.size(); ++i) {
    int channel = channels[i];
    uint16_t angle_ticks =
        decode_u16(data_span.subspan(i * sizeof(uint16_t), sizeof(uint16_t))
                       .first<sizeof(uint16_t)>());
    float angle_rad = static_cast<float>(angle_ticks) *
                      (2.0f * M_PI / 4096.0f); // Convert ticks to radians
    results.emplace_back(channel, angle_rad, angle_ticks);
  }

  data_cb(results);
}
