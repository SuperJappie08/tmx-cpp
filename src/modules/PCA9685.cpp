#include "modules/PCA9685.hpp"

PCA9685_module::PCA9685_module(
    std::function<void(std::vector<uint8_t>)> send_module) {
  this->send_module = send_module;
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