#include "modules/PCA9685.hpp"
#include <iostream>
PCA9685_module::PCA9685_module(uint8_t i2c_port, uint8_t address,
                               int frequency) {
  this->i2c_port = i2c_port;
  this->address = address;
  this->frequency = frequency;
  type = MODULE_TYPE::PCA9685;
}

bool PCA9685_module::set_pwm(uint8_t channel, uint16_t high, uint16_t low) {
  std::vector<uint8_t> data = {channel, (uint8_t)(low & 0xFF),
                               (uint8_t)(low >> 8), (uint8_t)(high & 0xFF),
                               (uint8_t)(high >> 8)};
  this->send_module(data);
  return true;
}

bool PCA9685_module::set_multiple_pwm(std::vector<PWM_val> pwm_vals) {
  std::vector<uint8_t> data;
  for (auto pwm_val : pwm_vals) {
    data.push_back(pwm_val.channel);
    data.push_back((uint8_t)(pwm_val.low & 0xFF));
    data.push_back((uint8_t)(pwm_val.low >> 8));
    data.push_back((uint8_t)(pwm_val.high & 0xFF));
    data.push_back((uint8_t)(pwm_val.high >> 8));
  }
  this->send_module(data);
  return true;
}

std::vector<uint8_t> PCA9685_module::init_data() {

  std::cout << "pca init data" << std::dec << (int)this->i2c_port << "A"
            << (int)this->address << "F" << (int)(this->frequency & 0xFF) << "F"
            << (int)(this->frequency >> 8) << std::endl;
  return {this->i2c_port, this->address, (uint8_t)(this->frequency & 0xFF),
          (uint8_t)(this->frequency >> 8)};
}

void PCA9685_module::data_callback(std::vector<uint8_t> data) {
  // it should not receive any data
  return;
}

void PCA9685_module::attach_send_module(
    std::function<void(std::vector<uint8_t>)> send_module) {
  this->send_module = send_module;
}