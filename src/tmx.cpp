#include "tmx_cpp/tmx.hpp"
#include <algorithm>
#include <cassert>
#include <iostream>
// #include <ranges>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <utility>

#include <tmx_cpp/message_types.hpp>
#include <tmx_cpp/serialization.hpp>
#include <tmx_cpp/types.hpp>
// #undef TMX_TX_DEBUG
#include <fstream>
using namespace tmx_cpp;

::std::ostream &operator<<(::std::ostream &out, const MESSAGE_TYPE &value) {
  return out << [value] {
#define PROCESS_VAL(p)                                                         \
  case (p):                                                                    \
    return #p;
    switch (value) {
      PROCESS_VAL(MESSAGE_TYPE::SERIAL_LOOP_BACK);
      PROCESS_VAL(MESSAGE_TYPE::SET_PIN_MODE);
      PROCESS_VAL(MESSAGE_TYPE::DIGITAL_WRITE);
      PROCESS_VAL(MESSAGE_TYPE::PWM_WRITE);
      PROCESS_VAL(MESSAGE_TYPE::MODIFY_REPORTING);
      PROCESS_VAL(MESSAGE_TYPE::FIRMWARE_VERSION);
      PROCESS_VAL(MESSAGE_TYPE::GET_PICO_UNIQUE_ID);
      PROCESS_VAL(MESSAGE_TYPE::SERVO_ATTACH);
      PROCESS_VAL(MESSAGE_TYPE::SERVO_WRITE);
      PROCESS_VAL(MESSAGE_TYPE::SERVO_DETACH);
      PROCESS_VAL(MESSAGE_TYPE::I2C_BEGIN);
      PROCESS_VAL(MESSAGE_TYPE::I2C_READ);
      PROCESS_VAL(MESSAGE_TYPE::I2C_WRITE);
      PROCESS_VAL(MESSAGE_TYPE::SONAR_NEW);
      PROCESS_VAL(MESSAGE_TYPE::DHT_NEW);
      PROCESS_VAL(MESSAGE_TYPE::STOP_ALL_REPORTS);
      PROCESS_VAL(MESSAGE_TYPE::ENABLE_ALL_REPORTS);
      PROCESS_VAL(MESSAGE_TYPE::RESET_DATA);
      PROCESS_VAL(MESSAGE_TYPE::RESET_BOARD);
      PROCESS_VAL(MESSAGE_TYPE::INITIALIZE_NEO_PIXELS);
      PROCESS_VAL(MESSAGE_TYPE::SHOW_NEO_PIXELS);
      PROCESS_VAL(MESSAGE_TYPE::SET_NEO_PIXEL);
      PROCESS_VAL(MESSAGE_TYPE::CLEAR_ALL_NEO_PIXELS);
      PROCESS_VAL(MESSAGE_TYPE::FILL_NEO_PIXELS);
      PROCESS_VAL(MESSAGE_TYPE::SPI_INIT);
      PROCESS_VAL(MESSAGE_TYPE::SPI_WRITE);
      PROCESS_VAL(MESSAGE_TYPE::SPI_READ);
      PROCESS_VAL(MESSAGE_TYPE::SPI_SET_FORMAT);
      PROCESS_VAL(MESSAGE_TYPE::SPI_CS_CONTROL);
      PROCESS_VAL(MESSAGE_TYPE::SET_SCAN_DELAY);
      PROCESS_VAL(MESSAGE_TYPE::ENCODER_NEW);
      PROCESS_VAL(MESSAGE_TYPE::SENSOR_NEW);
      PROCESS_VAL(MESSAGE_TYPE::PING);
      PROCESS_VAL(MESSAGE_TYPE::MODULE_NEW);
      PROCESS_VAL(MESSAGE_TYPE::MODULE_DATA);
      PROCESS_VAL(MESSAGE_TYPE::GET_ID);
      PROCESS_VAL(MESSAGE_TYPE::SET_ID);
      PROCESS_VAL(MESSAGE_TYPE::FEATURE_REQUEST);
      PROCESS_VAL(MESSAGE_TYPE::BOOTLOADER_RESET);
      PROCESS_VAL(MESSAGE_TYPE::MAX);

    default:
      return "UNKNOWN";
    }
#undef PROCESS_VAL
  }();
}
::std::ostream &operator<<(::std::ostream &out, const MESSAGE_IN_TYPE &value) {
  return out << [value] {
#define PROCESS_VAL(p)                                                         \
  case (p):                                                                    \
    return #p;
    switch (value) {
      PROCESS_VAL(MESSAGE_IN_TYPE::SERIAL_LOOP_BACK_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::DIGITAL_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::ANALOG_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::FIRMWARE_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::REPORT_PICO_UNIQUE_ID);
      PROCESS_VAL(MESSAGE_IN_TYPE::SERVO_UNAVAILABLE);
      PROCESS_VAL(MESSAGE_IN_TYPE::I2C_WRITE_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::I2C_READ_FAILED);
      PROCESS_VAL(MESSAGE_IN_TYPE::I2C_READ_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::SONAR_DISTANCE);
      PROCESS_VAL(MESSAGE_IN_TYPE::DHT_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::SPI_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::ENCODER_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::DEBUG_PRINT);
      PROCESS_VAL(MESSAGE_IN_TYPE::SENSOR_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::PONG_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::MODULE_MAIN_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::MODULE_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::GET_ID_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::SET_ID_REPORT);
      PROCESS_VAL(MESSAGE_IN_TYPE::FEATURE_REQUEST_REPORT);
    default:
      return "UNKNOWN";
    }
#undef PROCESS_VAL
  }();
}
TMX::TMX(std::function<void()> stop_func, std::string port,
         size_t parse_pool_size)
    : parsePool(std::max<size_t>(parse_pool_size, 1)), stop_func(stop_func),
      module_sys(std::make_shared<Modules>(this)),
      sensors_sys(std::make_shared<Sensors>(this)), is_stopped(false) {
  using namespace std::placeholders;

  this->serial = std::make_shared<CallbackAsyncSerial>(port, 115200);
  // sleep for a second
  this->serial->setCallback([](const char *data, size_t len) {
    std::cout << "TMX::callback: " << len << " bytes received, ignoring"
              << std::endl;
    // std::cout << "TMX::callback: " << len << " bytes received" << std::endl;
  });
  std::cout << "TMX: waiting for serial port to be ready" << std::endl;
  this->serial->write({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0}); // send a newline to the serial port to wake it up
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "TMX: done waiting for serial port to be ready" << std::endl;
// add to file
#if 0
  std::ofstream file("tmx_data.log", std::ios_base::app);
  file << "TMX: opening serial port: " << port << std::endl;
  file.flush();
  file.close();
#endif
  this->serial->setCallback(
      [this](const char *data, size_t len) { this->callback(data, len); });

  this->ping_thread = std::thread(&TMX::ping_task, this);
  this->add_callback(MESSAGE_IN_TYPE::PONG_REPORT,
                     std::bind(&TMX::ping_callback, this, _1));

  feature_detect_task();
}

TMX::~TMX() { this->stop(); }
#include <fstream>

void TMX::callback(const char *data, size_t len) {
#if 0
  // write to some file
  std::ofstream file("tmx_data.log", std::ios_base::app);
  file << "callback: len = " << len << " data: ";
  for (int i = 0; i < len; i++) {
    file << std::hex << (uint)data[i] << " ";
  }
  file << std::endl;
  file.flush();
  file.close();

  // binary file logging:
  std::ofstream bin_file("tmx_data.bin",
                         std::ios_base::app | std::ios_base::binary);
  bin_file.write(data, len);
  bin_file.flush();
  bin_file.close();

#endif
#ifdef TMX_TX_DEBUG
  std::cout << "callback" << std::endl;
  std::cout << "len =" << len << std::endl;
  std::cout << "data: ";

  for (int i = 0; i < len; i++) {
    std::cout << std::hex << (int)data[i] << " ";
  }
  std::cout << std::endl;
  std::cout.write(data, len);
  std::cout.flush();
#endif
  this->buffer.insert(this->buffer.end(), data, data + len);
  while (this->buffer.size() >= this->buffer[0] + 1) {
    this->parse(this->buffer);
  }
#ifdef TMX_TX_DEBUG
  std::cout << "end buffer size: " << this->buffer.size() << std::endl;
  std::cout << "end buffer: ";
  for (auto i : this->buffer) {
    std::cout << std::hex << (int)i << " ";
  }
  std::cout << "end buff " << std::endl;
#endif
}
void TMX::parse(std::vector<uint8_t> &buffer) {
  // length of the message: buffer[0]
  // message type: buffer[1]
  // message data: buffer[2] to buffer[buffer[0]]
  // std::cout << "parse start buffer = ";
  // for (auto i : buffer)
  // {
  //     std::cout << std::hex << (int)i << " ";
  // }
  // std::cout << "end buffer" << std::endl;
  auto N = buffer[0] + 1;
  if (buffer.size() < N) {
    return;
  }
  if (buffer.size() == N) {
    this->parseOne(buffer);
    buffer.clear();
    return;
  }
  auto subBuffer = std::vector<uint8_t>(buffer.begin(), buffer.begin() + N);

  buffer.erase(buffer.begin(), buffer.begin() + N);
  this->parseOne(subBuffer);
}
void TMX::parseOne(const std::vector<uint8_t> &message) {
#ifdef TMX_TX_DEBUG
  std::cout << "R charMessage = ";
  for (auto i : message) {
    std::cout << std::hex << (uint)(i & 0xFF) << " ";
  }
  std::cout << std::endl;
#endif

  boost::asio::post(this->parsePool,
                    std::bind(&TMX::parseOne_task, this, message));
}

void TMX::parseOne_task(const std::vector<uint8_t> &message) {
  // Note:: this runs on a different thread than any other things.
  // Makes it possible to have longer running callbacks without interfering with
  // other callbacks and reading in data.

  // msg: {len, type, ...}
  auto type = (MESSAGE_IN_TYPE)message[1];
#ifdef TMX_TX_DEBUG
  std::cout << "parseOne_task: type = " << type << std::endl;
#endif
  switch (type) {
  case MESSAGE_IN_TYPE::PONG_REPORT: {
    for (const auto &callback : this->ping_callbacks) {
      callback(message);
    }
  } break;
  case MESSAGE_IN_TYPE::FIRMWARE_REPORT:
    std::cout << "firmware version: " << (int)message[2] << '.'
              << (int)message[3] << std::endl;
    break;
  case MESSAGE_IN_TYPE::REPORT_PICO_UNIQUE_ID:
    std::cout << "pico unique id: ";
    for (int i = 2; i < 10; i++) {
      std::cout << std::hex << (int)message[i] << " ";
    }
    std::cout << std::endl;
    break;
  case MESSAGE_IN_TYPE::DIGITAL_REPORT: {
    auto pin = message[2];
    auto value = message[3];
    for (const auto &callback : this->digital_callbacks_pin) {
      if (callback.first == pin) {
        callback.second(pin, value);
      }
    }

    // call the callbacks with the normal message. Should use the other one for
    // normal use.
    for (const auto &callback : this->digital_callbacks) {
      callback(message);
    }
  } break;
  case MESSAGE_IN_TYPE::ANALOG_REPORT: {
    auto pin = message[2];
    auto value = decode_u16(std::span(message).subspan<3, sizeof(uint16_t)>());
    for (const auto &callback : this->analog_callbacks_pin) {
      if (callback.first == pin) {
        callback.second(pin, value);
      }
    }

    // call the callbacks with the normal message. Should use the other one for
    // normal use.
    for (const auto &callback : this->analog_callbacks) {
      callback(message);
    }
  } break;
  case MESSAGE_IN_TYPE::SERVO_UNAVAILABLE: {
    for (const auto &callback : this->servo_unavailable_callbacks) {
      callback(message);
    }
  } break;
  case MESSAGE_IN_TYPE::I2C_WRITE_REPORT: {
    auto msg_id = message[3];
    if (msg_id >= this->i2c_write_callbacks.size()) {
      std::cout << "I2C write callback not found: " << (int)msg_id << std::endl;
      return;
    }
    auto callback = this->i2c_write_callbacks[msg_id];
    if (callback.second) {
      callback.second->operator()(message);
    } else {
      std::cout << "I2C write callback not found: " << (int)msg_id << std::endl;
    }
    callback.second.reset();

    for (auto &callback : this->i2c_write_callbacks) {
      if (callback.first + 2 < std::time(nullptr)) {
        callback.second->operator()({
            0, 0, 0, 0, 0 // failed write callback as it took over 2sec.
        });
        callback.second.reset();
      }
    }

    // clear all callbacks if all are empty, to save memory
    // TODO: this is not thread safe, need to use mutex
    if (std::all_of(this->i2c_write_callbacks.begin(),
                    this->i2c_write_callbacks.end(),
                    [](const auto &callback) { return !callback.second; })) {
      this->i2c_write_callbacks.clear();
    }
  } break;
  case MESSAGE_IN_TYPE::I2C_READ_FAILED: { // TODO: this is not used anymore
    std::cout << "I2C read failed msg, but not used" << std::endl;
  } break;
  case MESSAGE_IN_TYPE::I2C_READ_REPORT: {
    auto msg_id = message[3];
    if (msg_id >= this->i2c_read_callbacks.size()) {
      std::cout << "I2C read callback not found: " << (int)msg_id << std::endl;
      return;
    }
    auto callback = this->i2c_read_callbacks[msg_id];
    if (callback.second) {
      callback.second->operator()(message);
    } else {
      std::cout << "I2C read callback not found: " << (int)msg_id << std::endl;
    }
    callback.second.reset();
    for (auto &callback : this->i2c_read_callbacks) {
      if (callback.first + 2 < std::time(nullptr)) {
        callback.second->operator()({
            0, 0, 0, 0, 0 // failed write callback as it took over 2sec.
        });
        callback.second.reset();
      }
    }
    // clear all callbacks if all are empty, to save memory
    // TODO: this is not thread safe, need to use mutex
    if (std::all_of(this->i2c_read_callbacks.begin(),
                    this->i2c_read_callbacks.end(),
                    [](const auto &callback) { return !callback.second; })) {
      this->i2c_read_callbacks.clear();
    }
  } break;
  case MESSAGE_IN_TYPE::SONAR_DISTANCE: {
    auto pin = message[2];

    // The distance value in centimeters, a left over from the original
    // Telemetrix protocol.
    auto value = decode_u16(std::span(message).subspan<3, sizeof(uint16_t)>());

    for (const auto &callback : this->sonar_callbacks_pin) {
      if (callback.first == pin) {
        callback.second(pin, value);
      }
    }
    for (const auto &callback : this->sonar_distance_callbacks) {
      callback(message);
    }
  } break;
  case MESSAGE_IN_TYPE::ENCODER_REPORT: {
    uint8_t pin = message[2];
    auto value = (int8_t)message[3];
    for (const auto &callback : this->encoder_callbacks_pin) {
      if (callback.first == pin) {
        callback.second(pin, value);
      }
    }
    for (const auto &callback : this->encoder_callbacks) {
      callback(message);
    }
  } break;
  case MESSAGE_IN_TYPE::DEBUG_PRINT: {
    for (const auto &callback : this->debug_print_callbacks) {
      callback(message);
    }
    uint16_t val = decode_u16(std::span(message).subspan<3, 2>());

    std::cout << "Debug print: " << std::dec << (uint)message[2] << " "
              << std::dec << (uint16_t)(val) << std::endl;
    // std::cout << "debug len:" << std::dec << (uint)message.size() <<
    // std::endl; for(auto i = 0; i < message.size(); i++) {
    //   std::cout << "debug " << i << ":" << std::dec << (uint)message[i] <<
    //   std::endl;
    // }
  } break;
  case MESSAGE_IN_TYPE::SERIAL_LOOP_BACK_REPORT:
    std::cout << "Serial loopback not implemented" << std::endl;
    break;
  case MESSAGE_IN_TYPE::DHT_REPORT: {
    for (const auto &callback : this->dht_callbacks) {
      callback(message);
    }
  } break;
  case MESSAGE_IN_TYPE::SPI_REPORT: {
    for (const auto &callback : this->spi_callbacks) {
      callback(message);
    }
  } break;
  case MESSAGE_IN_TYPE::SENSOR_MAIN_REPORT: {
    std::cout << "Sensor main report" << std::endl;
    auto type = message[2];
    if (type == 0) {
      // feature check
      auto feature = message[3];
      auto ok = message[4];
#ifdef TMX_TX_DEBUG
      if (ok) {
        std::cout << "Feature " << (int)feature << " is supported" << std::endl;
      } else {
        std::cout << "Feature " << (int)feature << " is not supported"
                  << std::endl;
      }
#endif
      this->sensors_sys->report_features(
          (SENSOR_TYPE)feature, ok,
          std::vector<uint8_t>(message.begin() + 3, message.end()));
    }
  } break;
  case MESSAGE_IN_TYPE::SENSOR_REPORT: {
    for (const auto &callback : this->sensor_callbacks) {
      callback(message);
    }
  } break;
  case MESSAGE_IN_TYPE::MODULE_MAIN_REPORT: {
    // std::cout << "Module main report" << std::endl;
    auto type = message[2];
    if (type == 0) {
      // feature check
      auto feature = message[3];
      auto ok = message[4];
#ifdef TMX_TX_DEBUG
      if (ok) {
        std::cout << "Feature " << (int)feature << " is supported" << std::endl;
      } else {
        std::cout << "Feature " << (int)feature << " is not supported"
                  << std::endl;
      }
#endif
      this->module_sys->report_features(
          (MODULE_TYPE)feature, ok,
          std::vector<uint8_t>(message.begin() + 3, message.end()));
    }
  } break;
  case MESSAGE_IN_TYPE::MODULE_REPORT: {
    for (const auto &callback : this->module_callbacks) {
      callback(message);
    }
  } break;
  case MESSAGE_IN_TYPE::GET_ID_REPORT: {
    std::cout << "Get id not implemented" << std::endl;
    std::cout << "ID = " << std::hex << (uint)message[2] << std::endl;
  } break;
  case MESSAGE_IN_TYPE::SET_ID_REPORT: {
    std::cout << "Set id not implemented" << std::endl;
    std::cout << "ID = " << std::hex << (uint)message[2] << std::endl;
  } break;
  case MESSAGE_IN_TYPE::FEATURE_REQUEST_REPORT: {
// msg type: message_type, okay, ... remaining options depending on type(sonar
// size, ...) print msg
#ifdef TMX_TX_DEBUG
    std::cout << "Feature request msg: ";
    for (auto i = 0; i < message.size(); i++) {
      std::cout << std::hex << (int)message[i] << " ";
    }
    std::cout << std::endl;
#endif
    auto msg_type = message[2];
    auto ok = message[3];
    while (this->features.size() < msg_type + 1) {
      // std::cout << "Feature request: " << (int)msg_type << " resize" <<
      // std::endl;
      this->features.push_back({0, {}});
    }
// std::cout << "Feature request: " << (int)msg_type << " ok:" << (int)ok <<
// std::endl; this->features[msg_type].clear();
#ifdef TMX_TX_DEBUG
    if (ok) {
      std::cout << "Feature request: " << (int)msg_type << " is ok"
                << std::endl;
    } else {
      std::cout << "Feature request: " << (int)msg_type << " is not ok"
                << std::endl;
    }
#endif
    this->features[msg_type].first = ok;
    this->features[msg_type].second = {message.begin() + 4, message.end()};
    // std::cout << "features value" << (int)this->features[msg_type].first <<
    // std::endl; std::cout << "Feature request done: " << (int)msg_type << " "
    // << (int)ok << std::endl;
    this->feature_index = msg_type;
    this->feature_cv.notify_all(); // notify other threads that a feature is
                                   // checked and that they can continue
    this->board_features.parse_features({message.begin() + 2, message.end()});
  } break;
  default:
    break;
  }
}

void TMX::sendPing(uint8_t num) {
  auto message =
      std::vector<uint8_t>{(uint8_t)MESSAGE_TYPE::PING, (uint8_t)num};
  this->sendMessage(message);

  // this->begin_time = clock();
}

/**
 * Send a message with some type. Length added automatically.
 */
void TMX::sendMessage(const std::vector<uint8_t> &message) {
  std::vector<char> charMessage(message.begin(), message.end());
  charMessage.insert(charMessage.begin(), charMessage.size());
  serial->write(charMessage);
#if 0
  std::ofstream file("tmx_data.log", std::ios_base::app);
  file << "writing: len = " << charMessage.size()
       << " command = " << (MESSAGE_TYPE)message[0] << " data: ";
  for (int i = 0; i < charMessage.size(); i++) {
    file << std::hex << (int)charMessage[i] << " ";
  }
  file << std::endl;
  file.flush();
  file.close();
#endif
}
/**
 * Send a message with some type. Length added automatically.
 */
void TMX::sendMessage(MESSAGE_TYPE type, const std::vector<uint8_t> &message) {
  if (type != MESSAGE_TYPE::FEATURE_REQUEST && !this->get_feature(type).first) {
    std::cout << "Feature not supported: " << type << std::endl;
    return;
  }
  std::vector<char> charMessage(message.begin(), message.end());
  charMessage.insert(charMessage.begin(),
                     {(char)(charMessage.size() + 1), (char)type});

#ifdef TMX_TX_DEBUG
  std::cout << "T charMessage = ";
  for (auto i : charMessage) {
    std::cout << std::hex << (uint)(i & 0xFF) << " ";
  }
  std::cout << std::endl;

#endif
#if 0
  std::ofstream file("tmx_data.log", std::ios_base::app);
  file << "writing here: len = " << charMessage.size()<< " command = " << type << " data: ";
  for (int i = 0; i < charMessage.size(); i++) {
    file << std::hex << (int)charMessage[i] << " ";
  }
  file << std::endl;
  file.flush();
  file.close();
#endif
  serial->write(charMessage);
}
void TMX::setPinMode(uint8_t pin, TMX::PIN_MODES mode, bool reporting,
                     uint16_t analog_differential) {
  std::vector<uint8_t> message;
  switch (mode) {
  case TMX::PIN_MODES::DIGITAL_INPUT:
  case TMX::PIN_MODES::DIGITAL_INPUT_PULL_DOWN:
  case TMX::PIN_MODES::DIGITAL_INPUT_PULL_UP:
    message = std::vector<uint8_t>{pin, mode, reporting};
    break;
  case TMX::PIN_MODES::DIGITAL_OUTPUT:
  case TMX::PIN_MODES::PWM_OUTPUT:
    message = std::vector<uint8_t>{pin, mode};
    break;
  case TMX::PIN_MODES::ANALOG_INPUT:
    pin -= this->board_features.analog_offset;

    if (pin < 0) {
      std::cout << "Analog pin out of range" << std::endl;
      return;
    }
    // if (pin > this->board_features.analog_pins) {
    //   std::cout << "Analog pin out of range" << std::endl;
    //   return;
    // }
    message = std::vector<uint8_t>{pin, mode};
    // message.reserve(message.size() + sizeof(uint16_t) + sizeof(reporting));

    append_range(message, encode_u16(analog_differential));
    message.push_back((uint8_t)reporting);

    break;
  default:
    break;
  }
  assert(message.size() >= 2);
  this->sendMessage(MESSAGE_TYPE::SET_PIN_MODE, message);
}

void TMX::digitalWrite(uint8_t pin, bool value) {
  std::vector<uint8_t> message = {pin, value};
  this->sendMessage(MESSAGE_TYPE::DIGITAL_WRITE, message);
}
void TMX::pwmWrite(uint8_t pin, uint16_t value) {
  std::vector<uint8_t> message = {pin};
  append_range(message, encode_u16(value));
  this->sendMessage(MESSAGE_TYPE::PWM_WRITE, message);
}
void TMX::add_callback(
    MESSAGE_IN_TYPE type,
    std::function<void(const std::vector<uint8_t> &)> callback) {
  switch (type) {
  case MESSAGE_IN_TYPE::PONG_REPORT:
    this->ping_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::FIRMWARE_REPORT:
    this->firmware_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::REPORT_PICO_UNIQUE_ID:
    this->pico_unique_id_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::SERIAL_LOOP_BACK_REPORT:
    this->serial_loop_back_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::DIGITAL_REPORT:
    this->digital_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::ANALOG_REPORT:
    this->analog_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::I2C_WRITE_REPORT:
    this->i2c_write_callbacks.push_back({0, callback});
    break;
    // case MESSAGE_IN_TYPE::I2C_READ_FAILED:
    //   this->i2c_read_failed_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::I2C_READ_REPORT:
    // this->i2c_read_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::SONAR_DISTANCE:
    this->sonar_distance_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::DHT_REPORT:
    this->dht_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::SPI_REPORT:
    this->spi_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::ENCODER_REPORT:
    this->encoder_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::DEBUG_PRINT:
    this->debug_print_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::SENSOR_REPORT:
    this->sensor_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::MODULE_REPORT:
    this->module_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::SERVO_UNAVAILABLE:
    this->servo_unavailable_callbacks.push_back(callback);
    break;
  case MESSAGE_IN_TYPE::GET_ID_REPORT:
  case MESSAGE_IN_TYPE::SET_ID_REPORT:
    std::cout << "Get id and set id not implemented" << std::endl;
    break;
  default:
    break;
  }
}

void TMX::add_digital_callback(uint8_t pin,
                               std::function<void(uint8_t, uint8_t)> callback) {
  this->digital_callbacks_pin.push_back({pin, callback});
}

void TMX::add_analog_callback(uint8_t pin,
                              std::function<void(uint8_t, uint16_t)> callback) {
  // FIXME: This is Pico specific code
  // if (pin < 26 || pin > 30) { // only pins 26-30 are analog
  //   return;
  // }
  this->analog_callbacks_pin.push_back({pin, callback});
  std::cout << "analog_callbacks_pin size = "
            << this->analog_callbacks_pin.size() << std::endl;
}

bool TMX::attach_encoder(uint8_t pin_A, uint8_t pin_B,
                         std::function<void(uint8_t, int8_t)> callback) {
  auto feature = this->get_feature(MESSAGE_TYPE::ENCODER_NEW);
  if (!feature.first) {
    std::cout << "encoders not supported by hw" << std::endl;
    return false;
  }
  if (feature.second.size() < 2) {
    std::cout << "Encoder not supported size err " << std::endl;
    return false;
  }
  if (this->board_features.max_encoders <= this->encoder_callbacks_pin.size()) {
    std::cout << "encoders at max capacity" << std::endl;
    return false;
  }

  this->encoder_callbacks_pin.push_back({pin_A, callback});
  uint8_t type = 2;                  // 'default' on quadrature encoder
  if (pin_B == 0xff || pin_B == 0) { // if pin_B is not set, go back to single
    pin_B = 0;
    type = 1;
  }

  if (type == 2 && this->board_features.encoder_dirs == 1) {
    std::cout << "hardware only supports single pin encoders, no quadrature"
              << std::endl;
    type = 1;
  }
  this->sendMessage(MESSAGE_TYPE::ENCODER_NEW, {type, pin_A, pin_B});
  return true;
}

bool TMX::attach_sonar(uint8_t trigger, uint8_t echo,
                       std::function<void(uint8_t, uint16_t)> callback) {

  auto feature = this->get_feature(MESSAGE_TYPE::SONAR_NEW);
  if (!feature.first) {
    std::cout << "Sonar not supported" << std::endl;
    return false;
  }
  if (feature.second.size() < 1) {
    std::cout << "Sonar not supported size err " << std::endl;
    return false;
  }
  if (feature.second[0] <= this->sonar_callbacks_pin.size()) {
    std::cout << "Sonar at max capacity" << std::endl;
    return false;
  }
  this->sonar_callbacks_pin.push_back({trigger, callback});
  this->sendMessage(MESSAGE_TYPE::SONAR_NEW, {trigger, echo});
  return true;
}

void TMX::attach_servo(uint8_t pin, uint16_t min_pulse, uint16_t max_pulse) {
  // Arduino expects 5 bytes [pin, min_h, min_l, max_h, max_l]
  std::vector<uint8_t> msg = {pin};
  // msg.reserve(5);

  append_range(msg, encode_u16(min_pulse));
  append_range(msg, encode_u16(max_pulse));

  this->sendMessage(MESSAGE_TYPE::SERVO_ATTACH, msg);
}

void TMX::write_servo(uint8_t pin, uint16_t duty_cycle) {
  std::vector<uint8_t> msg = {pin};
  // msg.reserve(3);

  append_range(msg, encode_u16(duty_cycle));
  std::cout << "write servo: " << (int)pin << " duty_cycle: " << (int)duty_cycle
            << std::endl;
  std::cout << "msg: ";
  for (auto i : msg) {
    std::cout << std::hex << (int)i << " ";
  }
  std::cout << std::endl;
  this->sendMessage(MESSAGE_TYPE::SERVO_WRITE, msg);
}

void TMX::detach_servo(uint8_t pin) {
  this->sendMessage(MESSAGE_TYPE::SERVO_DETACH, {pin});
}

void TMX::setScanDelay(uint8_t delay) {
  if (delay < 2) {
    delay = 2;
  }
  this->sendMessage(MESSAGE_TYPE::SET_SCAN_DELAY, {delay});
}

void TMX::stop() {
  if (this->is_stopped) {
    std::cout << "TMX: already stopped" << std::endl;
    return;
  }
  // this->sendMessage(MESSAGE_TYPE::STOP, {});
  this->is_stopped = true;
  this->parsePool.stop();
  this->feature_detect_thread.join();
  // trigger all conditions to stop waiting for features
  this->feature_cv.notify_all();
  if (this->ping_thread.joinable() &&
      std::this_thread::get_id() != this->ping_thread.get_id()) {
    this->ping_thread.join();
  }
  this->stop_func = []() {};
  this->parsePool.stop();
  this->parsePool.join();
  if (!this->serial) {
    return;
  }
  if (this->serial->isOpen()) {
    this->sendMessage(MESSAGE_TYPE::RESET_BOARD, {});
    this->serial->close();
  }
}

bool TMX::setI2CPins(uint8_t sda, uint8_t scl, uint8_t port) {
  // if (sda == 0 || scl == 0 || sda == scl) {
  //   // return false;
  // }
  if (sda == 0 || scl == 0) {
    if (port != 0 || sda != 0 || scl != 0) { // either all 0
      return false;
    }
  }
  const size_t MAX_I2C = 3;
  if (port >= MAX_I2C) {
    std::cout << "TMX: I2C port out of range" << std::endl;
    return false;
  }
  static bool initialized_ports[MAX_I2C] = {false, false,
                                            false}; // 2 ports for now
  if (initialized_ports[port]) {
    return false;
  }
  initialized_ports[port] = true;
  // TODO: add a check for pins, store some map of current pins
  this->sendMessage(MESSAGE_TYPE::I2C_BEGIN, {port, sda, scl});
  return true;
}

bool TMX::i2cWrite(uint8_t port, uint8_t address, std::vector<uint8_t> data,
                   std::function<void(bool, std::vector<uint8_t>)> callback,
                   bool nostop) {
  // this->i2c_write_callbacks.push_back(callback);
  auto msg_id = (uint8_t)this->i2c_write_callbacks.size();

  std::vector<uint8_t> message = {port, address, msg_id, (uint8_t)data.size(),
                                  nostop};
  append_range(message, data);
  this->sendMessage(MESSAGE_TYPE::I2C_WRITE, message);
  auto i2c_cb = [&callback](std::vector<uint8_t> msg) {
    std::cout << "I2C write callback" << std::endl;
    std::cout << "msg: ";
    for (auto i : msg) {
      std::cout << std::hex << (int)i << " ";
    }
    std::cout << std::endl;
    bool ok = msg[4];
    callback(ok, msg);
  };
  this->i2c_write_callbacks.push_back({std::time(nullptr), i2c_cb});
  return true;
}

bool TMX::i2cRead(uint8_t port, uint8_t address, uint8_t len,
                  std::vector<uint8_t> data,
                  std::function<void(bool, std::vector<uint8_t>)> callback) {
  // this->i2c_write_callbacks.push_back(callback);
  auto msg_id = (uint8_t)this->i2c_read_callbacks.size();

  std::vector<uint8_t> message = {
      port, address, msg_id,
      (uint8_t)data.size()}; // TODO: dit werkt niet op een pico, die kan alleen
                             // 1 register aan ipv langere berichten.
  append_range(message, data);
  this->sendMessage(MESSAGE_TYPE::I2C_READ, message);
  auto i2c_cb = [&callback](std::vector<uint8_t> msg) {
    std::cout << "I2C read callback" << std::endl;
    std::cout << "msg: ";
    for (auto i : msg) {
      std::cout << std::hex << (int)i << " ";
    }
    std::cout << std::endl;
    bool ok = msg[4];
    callback(ok, msg);
  };
  this->i2c_read_callbacks.push_back({std::time(nullptr), i2c_cb});
  return true;
}

std::pair<bool, std::vector<uint8_t>>
TMX::parse_buffer_for_message(std::vector<uint8_t> &buffer,
                              uint8_t wanted_len, // including length msgs
                              uint8_t wanted_type) {
  if (buffer.size() < wanted_len) {
    return {false, {}};
  }

  // expected message: {wanted_len, type, ...}
  // std::cout << "buffer size: " << buffer.size() << std::endl;
  while (buffer.size() >= wanted_len) {
    //   std::cout << "buffer size: " << buffer.size() << std::endl;
    //   std::cout << "buffer: ";
    //   for (auto i : buffer) {
    //     std::cout << std::hex << (int)i << " ";
    //   }
    auto len = buffer[0];
    //   std::cout << "len: " << (int)len << std::endl;
    if (len != wanted_len - 1) {      // different length message
      if (buffer.size() <= len + 1) { // not enough data
        return {false, {}};
      }
      buffer.erase(buffer.begin(),
                   buffer.begin() + 1 + len); // remove the message
      continue;
    }
    std::cout << "type: " << (int)buffer[1] << std::endl;
    std::cout << "id: " << (int)buffer[2] << std::endl;
    if (buffer[1] != wanted_type) {
      buffer.erase(buffer.begin(),
                   buffer.begin() + 1 + len); // remove the message
      continue;
    } else {
      return {true, {buffer.begin() + 1, buffer.begin() + wanted_len}};
    }
  }
  return {false, {}};
}

#include <thread>
bool TMX::check_port(const std::string &port) {
  std::future<bool> future = std::async(std::launch::async, [&port]() {
    try {
      auto serial = std::make_shared<CallbackAsyncSerial>(port, 115200);
      std::vector<uint8_t> buffer;
      serial->setCallback([&buffer](const char *data, size_t len) {
        // std::cout << "check port len: " << len << std::endl;
        buffer.insert(buffer.end(), data, data + len);
      });
      buffer.clear();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(2000)); // pico should respond within 100ms

      serial->write(
          {0, 0, 0, 0, 0, 0, 0, 1,
           (uint8_t)MESSAGE_TYPE::FIRMWARE_VERSION}); // send a get fw version
                                                      // message
      std::this_thread::sleep_for(
          std::chrono::milliseconds(2000)); // pico should respond within 100ms
      serial->close();
      std::cout << "buffer: ";
      for (auto i : buffer) {
        std::cout << std::hex << (int)i << " ";
      }
      std::cout << std::endl;
      auto out = TMX::parse_buffer_for_message(
          buffer, 4, (uint8_t)MESSAGE_IN_TYPE::FIRMWARE_REPORT);

      std::cout << "out: ";
      for (auto i : out.second) {
        std::cout << std::hex << (int)i << " ";
      }
      std::cout << std::endl;
      std::cout << "out size: " << out.second.size() << std::endl;
      std::cout << "out first: " << (int)out.first << std::endl;
      if (out.first) {
        std::cout << "check port: " << port << " is ok" << std::endl;
        std::cout << "returning true" << std::endl;
        return true;
      } else {
        std::cout << "check port: " << port << " is not ok" << std::endl;
        return false;
      }
    } catch (std::exception &e) {
      std::cout << "Exception" << e.what() << std::endl;
      return false;
    }
  });

  std::future_status status;

  status = future.wait_for(std::chrono::milliseconds(5000));
  std::cout << "status: " << (int)status << std::endl;
  if (status == std::future_status::timeout) {
    // verySlow() is not complete.
    std::cout << "check port: " << port << " timed out" << std::endl;
    return false;
  } else if (status == std::future_status::ready) {
    // verySlow() is complete.
    // Get result from future (if there's a need)
    auto ret = future.get();
    std::cout << "check port: " << port << " returned: " << ret << std::endl;
    return ret;
  }

  return false;
}

const std::vector<TMX::serial_port> TMX::accepted_ports = {
    {"", 0x1a86, 0x7523}, // CH340
    {"", 0x1a86, 0x7523}, // CH340
    {"", 0x2E8A, 0x000A}, // RP2040
    {"", 0x2E8A, 0x0009}, // RP2350
    {"", 0x2E8A, 0x00c0}, // Arduino RP pico
    {"", 0x239a, 0x802b}, // adafruit itsybitsy m4
    {"", 0x0483, 0x5740}, // stm32f103 blackpill
    // {"", 0x10c4, 0xea60}, // CP2102, dont use, its used by the lidar!!!
    // following ones are hallucinated by copilot, maybe check
    {"", 0x0403, 0x6001}, // FTDI FT232R
    {"", 0x0403, 0x6015}, // FTDI FT231X
    {"", 0x2341, 0x0042}, // Arduino Uno
    {"", 0x2341, 0x0243}, // Arduino Leonardo
    {"", 0x2341, 0x8036}, // Arduino Mega
    {"", 0x2341, 0x8037}, // Arduino Due
};

#include <boost/format.hpp> // std::format not yet supported
#include <filesystem>
#include <iostream>
#include <string>
#include <tmx_cpp/tmx_util.hpp>

#if !defined(TMX_CPP_WINDOWS)
std::vector<TMX::serial_port> TMX::get_available_ports() {
  std::vector<serial_port> port_names;
  namespace fs = std::filesystem;
  fs::path p("/dev/serial/by-id");
  try {
    if (!exists(p)) {
      std::cout << p.generic_string() << " does not exist" << std::endl;
      return port_names;
    } else {
      for (auto de : fs::directory_iterator(p)) {
        if (is_symlink(de.symlink_status())) {
          serial_port sp;
          fs::path symlink_points_at = read_symlink(de);
          fs::path canonical_path = fs::canonical(p / symlink_points_at);
          std::cout << canonical_path.generic_string() << std::endl;
          sp.port_name = canonical_path.generic_string();

          auto out = exec(
              (boost::format("udevadm info --name=%s | grep 'ID_VENDOR_ID'") %
               canonical_path.generic_string())
                  .str());

          out = out.substr(out.find('=') +
                           1); // ID_VENDOR_ID=0403, only get the 0403
          sp.pid = std::stoi(out, nullptr, 16); // convert to int
          out = exec(
              (boost::format("udevadm info --name=%s | grep ID_MODEL_ID=") %
               canonical_path.generic_string())
                  .str());
          out = out.substr(out.find('=') + 1);
          sp.vid = std::stoi(out, nullptr, 16);
          std::cout << sp.port_name << " " << sp.vid << ":" << sp.pid
                    << std::endl;
          port_names.push_back(sp);
        }
      }
    }
  } catch (const fs::filesystem_error &ex) {
    std::cout << ex.what() << '\n';
    throw port_names;
  }
  // std::sort(port_names.begin(), port_names.end());
  return port_names;
}
#endif

bool TMX::is_accepted_port(const serial_port &port) {
  for (const auto &accepted_port : TMX::accepted_ports) {
    if (accepted_port.vid == port.vid && accepted_port.pid == port.pid) {
      return true;
    }
  }
  return false;
}

uint8_t TMX::get_id(const TMX::serial_port &port) {
  auto serial = std::make_shared<CallbackAsyncSerial>(port.port_name, 115200);
  std::vector<uint8_t> buffer;
  serial->setCallback([&buffer](const char *data, size_t len) {
    std::cout << "get id len: " << len << std::endl;
    buffer.insert(buffer.end(), data, data + len);
  });
  buffer.clear();
  serial->write({1, (uint8_t)MESSAGE_TYPE::GET_ID}); // send a get id message
  std::this_thread::sleep_for(
      std::chrono::milliseconds(1000)); // pico should respond within 100ms
  serial->close();
  auto out = TMX::parse_buffer_for_message(
      buffer, 3, (uint8_t)MESSAGE_IN_TYPE::GET_ID_REPORT);
  if (out.first) {
    return out.second[1];
  } else {
    std::cout << "No id found" << std::endl;
    return 0xfe;
  };
}

bool TMX::set_id(const TMX::serial_port &port, uint8_t id) {
  auto serial = std::make_shared<CallbackAsyncSerial>(port.port_name, 115200);
  std::vector<uint8_t> buffer;
  serial->setCallback([&buffer](const char *data, size_t len) {
    std::cout << "set id len: " << len << std::endl;
    buffer.insert(buffer.end(), data, data + len);
  });
  buffer.clear();
  std::cout << "setting to id: " << (int)id << std::endl;
  serial->write(
      {2, (uint8_t)MESSAGE_TYPE::SET_ID, (char)id}); // send a set id message
  std::this_thread::sleep_for(
      std::chrono::milliseconds(2000)); // pico should respond within 100ms
  serial->close();
  if (buffer.size() < 3) {
    std::cout << "no data" << std::endl;
    return false;
  }
  std::cout << "set id data" << std::endl;
  for (auto i : buffer) {
    std::cout << std::hex << (int)i << " ";
  }
  std::cout << std::endl;
  auto out = TMX::parse_buffer_for_message(
      buffer, 3, (uint8_t)MESSAGE_IN_TYPE::SET_ID_REPORT);
  // expected message: {2, MESSAGE_IN_TYPE::SET_ID_REPORT, id}
  return out.first && out.second[1] == id;
}

void TMX::ping_task() {
  std::cout << "ping task started" << std::endl;
  this->get_feature(MESSAGE_TYPE::BOOTLOADER_RESET);
  if (!this->get_feature(MESSAGE_TYPE::PING).first) {
    std::cout << "ping not supported" << std::endl;
    return;
  }
  uint8_t num = 0;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (!this->is_stopped) {
    num++;
    this->sendPing(num);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    if ((num - this->last_ping) > 10) {
      std::cout << "\033[1;31mTelemetrix stopped due to missed pings | Missed: "
                << ((int)((num - this->last_ping))) << "\033[0m" << std::endl;
      this->stop_func();
    }
  }
}

void TMX::ping_callback(const std::vector<uint8_t> message) {
  if (this->first_magic) {
    this->magic = message[3];
    this->first_magic = false;
  }
  if (this->magic != message[3]) {
    std::cout << "Magic changed" << (int)this->magic << "-> " << (int)message[3]
              << std::endl;
    this->stop_func();
  }
  uint8_t ping_reply = message[2];
  if (ping_reply - 1 != this->last_ping) {
    std::cout << "Ping reply mismatch: " << (int)ping_reply
              << " != " << (int)(this->last_ping + 1) << std::endl;
    // this->stop_func();
  }
  this->last_ping = ping_reply;
}

void TMX::feature_detect_task() {
  this->feature_detected = false;

  for (auto i = 0; i < (int)MESSAGE_TYPE::MAX && !this->is_stopped; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    this->sendMessage(MESSAGE_TYPE::FEATURE_REQUEST, {(uint8_t)i});
  }
  if (this->is_stopped) {
    return;
  }
  {
    // wait for mutex
    std::unique_lock<std::mutex> lk(this->feature_mutex);
#ifdef TMX_TX_DEBUG
    std::cout << "Waiting main " << this->feature_index << "..."
              << ((int)MESSAGE_TYPE::MAX - 1) << std::endl;
#endif
    this->feature_cv.wait(lk, [this] {
      return this->feature_index >= ((int)MESSAGE_TYPE::MAX - 1);
    });
    TMX_DEBUG std::cout << "done waiting main " << this->feature_index
                        << std::endl;
  }
  TMX_DEBUG std::cout << "Feature detect done" << std::endl;
  this->feature_detected = true;

  this->module_sys->check_features();
  this->sensors_sys->check_features();
}

std::pair<bool, std::vector<uint8_t>> TMX::get_feature(MESSAGE_TYPE type) {
  if (!this->feature_detected && this->feature_index < (int)type) {
    std::unique_lock<std::mutex> lk(this->feature_mutex);
    TMX_DEBUG std::cout << "Waiting " << (int)type << " " << this->feature_index
                        << "... \n";
    this->feature_cv.wait(lk, [this, type] {
      return this->feature_index >= (int)type || this->feature_detected ||
             this->is_stopped;
    });
    TMX_DEBUG std::cout << "done waiting " << (int)type << " "
                        << this->feature_index << std::endl;
  }
  if (this->features.size() < (int)type) {
    return {false, {}};
  }
  return this->features[(int)type];
}
