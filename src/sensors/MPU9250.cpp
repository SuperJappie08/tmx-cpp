#include <cassert>

#include <tmx_cpp/tmx_util.hpp>
#include <tmx_cpp/sensors/MPU9250.hpp>

using namespace tmx_cpp;

MPU9250_module::MPU9250_module(uint8_t i2c_port, uint8_t address, MPU9250_cb_t data_cb)
: data_cb(data_cb), i2c_port(i2c_port), address(address) {
  this->type = SENSOR_TYPE::MPU9250;
}

std::vector<uint8_t> MPU9250_module::init_data() {
  this->type = SENSOR_TYPE::MPU9250;

  return {i2c_port , address};
}

void MPU9250_module::data_callback(std::vector<uint8_t> data) {
  static_assert(sizeof(float) == sizeof(uint32_t));
  static_assert(sizeof(float) == 4);
  // 3 measurements x 3 axes + Quaterion (4)
  assert(data.size() == sizeof(float) * (3 * 3 + 4));

  std::vector<float> acceleration;
  std::vector<float> gyro;
  std::vector<float> magnetic_field;
  std::vector<float> quaternion;

  for (int acc_idx = 0; acc_idx < 3; acc_idx++) {
    acceleration.push_back(decode_float(std::vector(
      data.cbegin() + acc_idx * sizeof(float), data.cbegin() + (acc_idx + 1) * sizeof(float))));
  }

  for (int gyro_idx = 3; gyro_idx < 6; gyro_idx++) {
    gyro.push_back(decode_float(std::vector(
      data.cbegin() + gyro_idx * sizeof(float), data.cbegin() + (gyro_idx + 1) * sizeof(float))));
  }

  for (int mag_idx = 6; mag_idx < 9; mag_idx++) {
    magnetic_field.push_back(decode_float(std::vector(
      data.cbegin() + mag_idx * sizeof(float), data.cbegin() + (mag_idx + 1) * sizeof(float))));
  }

  for (int q_idx = 9; q_idx < 13; q_idx++) {
    quaternion.push_back(decode_float(std::vector(data.cbegin()+q_idx*sizeof(float),data.cbegin() + (q_idx+1)*sizeof(float))));
  }

  assert(acceleration.size() == 3);
  assert(gyro.size() == 3);
  assert(magnetic_field.size() == 3);
  assert(quaternion.size() == 4);

  data_cb(acceleration, gyro, magnetic_field, quaternion);
}
