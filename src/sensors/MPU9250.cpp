#include <cassert>
#include <ranges>
#include <span>

#include <tmx_cpp/sensors/MPU9250.hpp>
#include <tmx_cpp/serialization.hpp>

using namespace tmx_cpp;

MPU9250_module::MPU9250_module(uint8_t i2c_port, uint8_t address, MPU9250_cb_t data_cb)
: data_cb(data_cb), i2c_port(i2c_port), address(address) {
  this->type = SENSOR_TYPE::MPU9250;
}

std::vector<uint8_t> MPU9250_module::init_data() {
  this->type = SENSOR_TYPE::MPU9250;

  return {i2c_port, address};
}

void MPU9250_module::data_callback(std::vector<uint8_t> data) {
  static_assert(sizeof(float) == sizeof(uint32_t));
  static_assert(sizeof(float) == 4);
  // 3 measurements x 3 axes + Quaterion (4)
  assert(data.size() == sizeof(float) * (3 * 3 + 4));
  auto data_span = std::span(data);

  std::array<float, 3> acceleration;
  std::array<float, 3> gyro;
  std::array<float, 3> magnetic_field;
  std::array<float, 4> quaternion;

  for (int acc_idx = 0; acc_idx < 3; acc_idx++) {
    acceleration[acc_idx] =
      decode_float(data_span.subspan(acc_idx * sizeof(float)).first<sizeof(float)>());
  }

  for (int gyro_idx = 3; gyro_idx < 6; gyro_idx++) {
    gyro[gyro_idx - 3] =
      decode_float(data_span.subspan(gyro_idx * sizeof(float)).first<sizeof(float)>());
  }

  for (int mag_idx = 6; mag_idx < 9; mag_idx++) {
    magnetic_field[mag_idx - 6] =
      decode_float(data_span.subspan(mag_idx * sizeof(float)).first<sizeof(float)>());
  }

  for (int q_idx = 9; q_idx < 13; q_idx++) {
    quaternion[q_idx - 9] =
      decode_float(data_span.subspan(q_idx * sizeof(float)).first<sizeof(float)>());
  }

  data_cb(acceleration, gyro, magnetic_field, quaternion);
}
