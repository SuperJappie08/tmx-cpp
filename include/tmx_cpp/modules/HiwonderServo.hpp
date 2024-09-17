#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <string>

#include "tmx_cpp/modules/Module_t.hpp"

namespace tmx_cpp {

class HiwonderServo_module : public Module_type {
public:
  std::function<void(std::vector<uint8_t>)> send_module;
  std::vector<uint8_t> servo_ids;
  enum HIWONDER_SERVO_COMMANDS : uint8_t {
    SET_SERVO = 1, // 2nd byte is the number of servos
    SET_ENABLE = 2,
    SET_ID = 3,
    VERIFY_ID = 4,
    SET_RANGE = 5,
    GET_RANGE = 6,
    SET_OFFSET = 7,
    GET_OFFSET = 8,
    SET_VOLTAGE_RANGE = 9,
  };
  enum HIWONDER_SERVO_RESPONSES : uint8_t {
    SERVO_POSITION = 0,
    SERVO_VERIFY = HIWONDER_SERVO_COMMANDS::VERIFY_ID,
    SERVO_RANGE = HIWONDER_SERVO_COMMANDS::GET_RANGE,
    SERVO_OFFSET = HIWONDER_SERVO_COMMANDS::GET_OFFSET,
  };
  struct Servo_pos {
    uint8_t id;
    uint16_t angle;
    uint16_t time;
  };
  HiwonderServo_module(uint8_t uart_port, uint8_t rx_pin, uint8_t tx_pin,
                       std::vector<uint8_t> servo_ids,
                       std::function<void(std::vector<Servo_pos>)> position_cb,
                       std::function<void(int, bool)> verify_cb,
                       std::function<void(int, uint16_t, uint16_t)> range_cb,
                       std::function<void(int, uint16_t)> offset_cb

  ); // + some callbacks
  // bool set_pwm(uint8_t channel, uint16_t high, uint16_t low=0);
  // struct PWM_val {
  //     uint8_t channel;
  //     uint16_t high;
  //     uint16_t low = 0;
  // };
  // bool set_multiple_pwm(std::vector<PWM_val> pwm_vals);
  void data_callback(
      std::vector<uint8_t> data); // when receiving data from the servos back

  bool set_single_servo(uint8_t servo_id, uint16_t angle, uint16_t time = 100);
  bool set_multiple_servos(std::vector<std::pair<uint8_t, uint16_t>> servo_vals,
                           uint16_t time = 100);
  bool set_enable_servo(uint8_t servo_id, bool enable);
  bool set_enabled_all(bool enable);
  bool set_id(uint8_t new_id, uint8_t old_id = 0xFF);
  bool verify_id(uint8_t id);
  bool set_range(uint8_t servo_id, uint16_t min, uint16_t max);
  bool set_voltage_range(uint8_t servo_id, float min, float max);
  bool set_offset(uint8_t servo_id, uint16_t offset);
  bool get_range(uint8_t servo_id);
  bool get_offset(uint8_t servo_id);

  uint8_t get_servo_num(uint8_t servo_id);

  std::vector<uint8_t> init_data();
  std::function<void(std::vector<Servo_pos>)> position_cb;
  std::function<void(int, bool)> verify_cb;
  std::function<void(int, uint16_t, uint16_t)> range_cb;
  std::function<void(int, uint16_t)> offset_cb;
  void
  attach_send_module(std::function<void(std::vector<uint8_t>)> send_module);

private:
  uint8_t uart_port;
  uint8_t rx_pin;
  uint8_t tx_pin;
};

}