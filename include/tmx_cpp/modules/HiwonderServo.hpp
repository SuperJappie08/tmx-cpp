#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <algorithm>
#include <cassert>
#include <functional>
#include <future>
#include <iostream>
#include <optional>
#include <string>
#include <tuple>

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
    MOTOR_MODE_WRITE = 10,
    ADD_SERVO = 11,
  };
  enum HIWONDER_SERVO_RESPONSES : uint8_t {
    SERVO_POSITION = 0,
    SERVO_VERIFY = HIWONDER_SERVO_COMMANDS::VERIFY_ID,
    SERVO_RANGE = HIWONDER_SERVO_COMMANDS::GET_RANGE,
    SERVO_OFFSET = HIWONDER_SERVO_COMMANDS::GET_OFFSET,
    SERVO_ADDED = HIWONDER_SERVO_COMMANDS::ADD_SERVO,
  };
  struct Servo_pos {
    uint8_t id;
    uint16_t angle;
    uint16_t time;
  };
  HiwonderServo_module(
      uint8_t uart_port, uint8_t rx_pin, uint8_t tx_pin, std::vector<uint8_t> servo_ids,
      std::function<void(std::vector<std::tuple<uint8_t, Servo_pos>>)> position_cb);

  std::optional<uint8_t> register_servo_id(uint8_t servo_id);

  void data_callback(std::vector<uint8_t> data); // when receiving data from the servos back

  bool set_single_servo(uint8_t servo_id, uint16_t angle, uint16_t time = 100);
  bool set_multiple_servos(std::vector<std::pair<uint8_t, uint16_t>> servo_vals,
                           uint16_t time = 100);
  bool set_enable_servo(uint8_t servo_id, bool enable);
  bool set_enabled_all(bool enable);
  bool set_id(uint8_t new_id, uint8_t old_id = 0xFF);
  bool verify_id(uint8_t servo_id);
  bool set_range(uint8_t servo_id, uint16_t min, uint16_t max);
  bool set_voltage_range(uint8_t servo_id, float min, float max);
  bool set_offset(uint8_t servo_id, int16_t offset);
  std::optional<std::tuple<uint16_t, uint16_t>> get_range(uint8_t servo_id);
  std::optional<int16_t> get_offset(uint8_t servo_id);
  bool motor_mode_write(uint8_t servo_id, int16_t speed);
  uint8_t get_servo_num(uint8_t servo_id);

  std::vector<uint8_t> init_data();
  std::function<void(std::vector<std::tuple<uint8_t, Servo_pos>>)> position_cb;
  void attach_send_module(std::function<void(std::vector<uint8_t>)> send_module);

private:
  uint8_t uart_port;
  uint8_t rx_pin;
  uint8_t tx_pin;

  // std::optional<std::promise<void>> position_promise;
  std::optional<std::promise<std::tuple<uint8_t, bool>>> verify_id_promise;
  std::optional<std::promise<std::tuple<uint8_t, std::tuple<uint16_t, uint16_t>>>> range_promise;
  std::optional<std::promise<std::tuple<uint8_t, int16_t>>> offset_promise;
  std::optional<std::promise<std::tuple<uint8_t, uint8_t>>> add_servo_promise;
};

} // namespace tmx_cpp
