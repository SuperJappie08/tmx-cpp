#pragma once
#include "tmx_cpp/modules/Module_t.hpp"
#include <functional>
#include <stdint.h>
#include <vector>

namespace tmx_cpp {

class PCA9685_module : public Module_type {
public:
  std::function<void(std::vector<uint8_t>)> send_module;
  PCA9685_module(uint8_t i2c_port, uint8_t address = 0x40, int frequency = 200);
  bool set_pwm(uint8_t channel, uint16_t high, uint16_t low = 0);
  struct PWM_val {
    uint8_t channel;
    uint16_t high;
    uint16_t low = 0;
  };
  bool set_multiple_pwm(std::vector<PWM_val> pwm_vals);

  bool set_mircoseconds(uint8_t channel, uint16_t microseconds);

  std::vector<uint8_t> init_data();
  void data_callback(std::vector<uint8_t> data);
  void attach_send_module(std::function<void(std::vector<uint8_t>)> send_module);

private:
  uint8_t i2c_port;
  uint8_t address;
  int frequency;
};

} // namespace tmx_cpp