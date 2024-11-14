#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "async_serial/AsyncSerial.h"

#include "tmx_cpp/modules.hpp"
#include "tmx_cpp/sensors.hpp"
#include "tmx_cpp/types.hpp"

namespace tmx_cpp {

using callback_func = std::function<void(std::vector<uint8_t>)>;
using callback_vec = std::vector<callback_func>;
using callback_func_pin = std::function<void(uint8_t, uint8_t)>;
using callback_func_pin16 = std::function<void(uint8_t, uint16_t)>;
using callback_func_pin_int = std::function<void(uint8_t, int8_t)>;

class TMX {
public:
  enum MESSAGE_IN_TYPE : uint8_t;
  enum MESSAGE_TYPE : uint8_t;

public:
  std::vector<uint8_t> buffer;
  // normal callbacks
  callback_vec ping_callbacks;
  callback_vec firmware_callbacks;
  callback_vec pico_unique_id_callbacks;
  callback_vec digital_callbacks;
  callback_vec analog_callbacks;
  callback_vec servo_unavailable_callbacks;
  callback_vec i2c_write_callbacks;
  callback_vec i2c_read_failed_callbacks;
  callback_vec i2c_read_callbacks;
  callback_vec sonar_distance_callbacks;
  callback_vec dht_callbacks;
  callback_vec spi_callbacks;
  callback_vec encoder_callbacks;
  callback_vec debug_print_callbacks;
  callback_vec sensor_callbacks;
  callback_vec module_callbacks;
  callback_vec serial_loop_back_callbacks;
  // callbacks with a specific pin:
  std::vector<std::pair<uint8_t, callback_func_pin>> digital_callbacks_pin;
  std::vector<std::pair<uint8_t, callback_func_pin16>> analog_callbacks_pin;
  std::vector<std::pair<uint8_t, callback_func_pin_int>> encoder_callbacks_pin; // todo check types
  std::vector<std::pair<uint8_t, callback_func_pin16>> sonar_callbacks_pin;

  void add_callback(MESSAGE_IN_TYPE type,
                    std::function<void(const std::vector<uint8_t> &)> callback);
  void add_digital_callback(uint8_t pin, std::function<void(uint8_t, uint8_t)> callback);
  void add_analog_callback(uint8_t pin, std::function<void(uint8_t, uint16_t)> callback);

  void parse(std::vector<uint8_t> &buffer);
  void parseOne(const std::vector<uint8_t> &buffer);
  void parseOne_task(const std::vector<uint8_t> &buffer);
  boost::asio::thread_pool parsePool;
  void stop();
  // Sensors sensors;

public:
  TMX(std::function<void()> stop_func, std::string port = "/dev/ttyACM0");
  ~TMX();
  enum MESSAGE_TYPE : uint8_t {
    SERIAL_LOOP_BACK = 0,
    SET_PIN_MODE = 1,
    DIGITAL_WRITE = 2,
    PWM_WRITE = 3,
    MODIFY_REPORTING = 4,
    FIRMWARE_VERSION = 5,
    GET_PICO_UNIQUE_ID = 6,
    SERVO_ATTACH = 7,
    SERVO_WRITE = 8,
    SERVO_DETACH = 9,
    I2C_BEGIN = 10,
    I2C_READ = 11,
    I2C_WRITE = 12,
    SONAR_NEW = 13,
    DHT_NEW = 14,
    STOP_ALL_REPORTS = 15,
    ENABLE_ALL_REPORTS = 16,
    RESET_DATA = 17,
    RESET_BOARD = 18,
    INITIALIZE_NEO_PIXELS = 19,
    SHOW_NEO_PIXELS = 20,
    SET_NEO_PIXEL = 21,
    CLEAR_ALL_NEO_PIXELS = 22,
    FILL_NEO_PIXELS = 23,
    SPI_INIT = 24,
    SPI_WRITE = 25,
    SPI_READ = 26,
    SPI_SET_FORMAT = 27,
    SPI_CS_CONTROL = 28,
    SET_SCAN_DELAY = 29,
    ENCODER_NEW = 30,
    SENSOR_NEW = 31,
    PING = 32,
    MODULE_NEW = 33,
    MODULE_DATA = 34,
    GET_ID = 35,
    SET_ID = 36,
  };
  enum MESSAGE_IN_TYPE : uint8_t {
    SERIAL_LOOP_BACK_REPORT = 0,
    DIGITAL_REPORT = 2,
    ANALOG_REPORT = 3,
    FIRMWARE_REPORT = MESSAGE_TYPE::FIRMWARE_VERSION,
    REPORT_PICO_UNIQUE_ID = 6,
    SERVO_UNAVAILABLE = 7, // for the future
    I2C_WRITE_REPORT = 8,
    I2C_READ_FAILED = 9,
    I2C_READ_REPORT = 10,
    SONAR_DISTANCE = 11,
    DHT_REPORT = 12,
    SPI_REPORT = 13,
    ENCODER_REPORT = 14,
    DEBUG_PRINT = 99,
    SENSOR_REPORT = 20,
    PONG_REPORT = 32,
    MODULE_REPORT = 34,
    GET_ID_REPORT = MESSAGE_TYPE::GET_ID,
    SET_ID_REPORT = MESSAGE_TYPE::SET_ID,
  };
  enum PIN_MODES : uint8_t {
    DIGITAL_INPUT = 0,
    DIGITAL_OUTPUT = 1,
    PWM_OUTPUT = 2,
    DIGITAL_INPUT_PULL_UP = 3,
    DIGITAL_INPUT_PULL_DOWN = 4,
    ANALOG_INPUT = 5,
  };
  void callback(const char *data, size_t len);
  void sendPing(uint8_t num = 0);
  void sendMessage(const std::vector<uint8_t> &message);
  void sendMessage(TMX::MESSAGE_TYPE type, const std::vector<uint8_t> &message);
  // Normal functions for use by the user:
  void setPinMode(uint8_t pin, TMX::PIN_MODES mode, bool reporting = true,
                  uint16_t analog_differential = 0);
  void digitalWrite(uint8_t pin, bool value);
  void pwmWrite(uint8_t pin, uint16_t value);

  void attach_encoder(uint8_t pin_A, uint8_t pin_B, callback_func_pin_int callback);

  /// The sonar callback and return value is in centimeters, as specified by the original telemetrix
  /// protocol.
  void attach_sonar(uint8_t trigger, uint8_t echo, std::function<void(uint8_t, uint16_t)> callback);

  // TODO: Maybe add angle remapping
  void attach_servo(uint8_t pin, uint16_t min_pulse = 1000, uint16_t max_pulse = 2000);
  void write_servo(uint8_t pin, uint16_t duty_cycle);
  void detach_servo(uint8_t pin);

  void setScanDelay(uint8_t delay);
  bool setI2CPins(uint8_t sda, uint8_t scl, uint8_t port);
  std::shared_ptr<CallbackAsyncSerial> serial;

  static bool check_port(const std::string &port);
  struct serial_port {
    std::string port_name;
    uint pid;
    uint vid;
  };
  static const std::vector<serial_port> accepted_ports;

  static std::vector<serial_port> get_available_ports();
  static bool is_accepted_port(const serial_port &port);
  static uint8_t get_id(const serial_port &port);
  static bool set_id(const serial_port &port, uint8_t id);
  static std::pair<bool, std::vector<uint8_t>>
  parse_buffer_for_message(std::vector<uint8_t> &buffer, uint8_t len, uint8_t type);

private:                   /* Ping related elements */
  std::thread ping_thread; // TODO: jthread from c++20
  uint8_t last_ping = 0;
  uint8_t magic = 0;
  bool first_magic = true;
  bool is_stopped = false;

  void ping_task();
  void ping_callback(const std::vector<uint8_t> msg);
  std::function<void()> stop_func;
};

} // namespace tmx_cpp