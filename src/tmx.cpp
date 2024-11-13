#include "tmx_cpp/tmx.hpp"
#include <algorithm>
#include <cassert>
#include <iostream>
// #include <ranges>
#include <boost/thread.hpp>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <utility>

#include <tmx_cpp/serialization.hpp>

#ifdef TMX_HW_DEBUG
#define POOL_SIZE 1
#else
#define POOL_SIZE boost::thread::hardware_concurrency()
#endif

using namespace tmx_cpp;

TMX::TMX(std::function<void()> stop_func, std::string port)
    : parsePool(POOL_SIZE), stop_func(stop_func) {
  this->serial = std::make_shared<CallbackAsyncSerial>(port, 115200);
  this->serial->setCallback([this](const char *data, size_t len) { this->callback(data, len); });

  this->ping_thread = std::thread(&TMX::ping_task, this);
  this->add_callback(MESSAGE_IN_TYPE::PONG_REPORT,
                     std::bind(&TMX::ping_callback, this, std::placeholders::_1));
  // this->add_callback(
  //   MESSAGE_IN_TYPE::ANALOG_REPORT, [](std::vector<uint8_t> t) { t[1] = 3; });
}

TMX::~TMX() { this->stop(); }

void TMX::callback(const char *data, size_t len) {
  // std::cout << "callback" << std::endl;
  // std::cout << "len =" << len << std::endl;
  // std::cout << "data: ";

  // for (int i = 0; i < len; i++)
  // {
  //     std::cout << std::hex << (int)data[i] << " ";
  // }
  // std::cout << std::endl;
  // std::cout.write(data, len);
  // std::cout.flush();
  this->buffer.insert(this->buffer.end(), data, data + len);
  while (this->buffer.size() >= this->buffer[0] + 1) {
    this->parse(this->buffer);
  }
  // std::cout << "end buffer size: " << this->buffer.size() << std::endl;
  // std::cout << "end buffer: ";
  // for (auto i : this->buffer)
  // {
  //     std::cout << std::hex << (int)i << " ";
  // }
  // std::cout << "end buff " << std::endl;
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
#ifdef TMX_RX_DEBUG
  std::cout << "R charMessage = ";
  for (auto i : message) {
    std::cout << std::hex << (uint)(i & 0xFF) << " ";
  }
  std::cout << std::endl;
#endif

  boost::asio::post(this->parsePool, std::bind(&TMX::parseOne_task, this, message));
}

void TMX::parseOne_task(const std::vector<uint8_t> &message) {
  // Note:: this runs on a different thread than any other things.
  // Makes it possible to have longer running callbacks without interfering with
  // other callbacks and reading in data.

  auto type = (TMX::MESSAGE_IN_TYPE)message[1];
  switch (type) {
  case TMX::MESSAGE_IN_TYPE::PONG_REPORT: {
    for (const auto &callback : this->ping_callbacks) {
      callback(message);
    }
  } break;
  case TMX::MESSAGE_IN_TYPE::FIRMWARE_REPORT:
    std::cout << "firmware version: " << (int)message[2] << '.' << (int)message[3] << std::endl;
    break;
  case TMX::MESSAGE_IN_TYPE::REPORT_PICO_UNIQUE_ID:
    std::cout << "pico unique id: ";
    for (int i = 2; i < 10; i++) {
      std::cout << std::hex << (int)message[i] << " ";
    }
    std::cout << std::endl;
    break;
  case TMX::MESSAGE_IN_TYPE::DIGITAL_REPORT: {
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
  case TMX::MESSAGE_IN_TYPE::ANALOG_REPORT: {
    auto pin = message[2];
    auto value = decode_u16(std::span(message).subspan<3, sizeof(uint16_t)>());
    for (const auto &callback : this->analog_callbacks_pin) {
      if (callback.first == pin) {
        callback.second(pin + 26, value);
      }
    }

    // call the callbacks with the normal message. Should use the other one for
    // normal use.
    for (const auto &callback : this->analog_callbacks) {
      callback(message);
    }
  } break;
  case TMX::MESSAGE_IN_TYPE::SERVO_UNAVAILABLE: {
    for (const auto &callback : this->servo_unavailable_callbacks) {
      callback(message);
    }
  } break;
  case TMX::MESSAGE_IN_TYPE::I2C_WRITE_REPORT: {
    for (const auto &callback : this->i2c_write_callbacks) {
      callback(message);
    }
  } break;
  case TMX::MESSAGE_IN_TYPE::I2C_READ_FAILED: {
    for (const auto &callback : this->i2c_read_failed_callbacks) {
      callback(message);
    }
  } break;
  case TMX::MESSAGE_IN_TYPE::I2C_READ_REPORT: {
    for (const auto &callback : this->i2c_read_callbacks) {
      callback(message);
    }
  } break;
  case TMX::MESSAGE_IN_TYPE::SONAR_DISTANCE: {
    auto pin = message[2];

    // The distance value in centimeters, a left over from the original Telemetrix protocol.
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
  case TMX::MESSAGE_IN_TYPE::ENCODER_REPORT: {
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
  case TMX::MESSAGE_IN_TYPE::DEBUG_PRINT: {
    for (const auto &callback : this->debug_print_callbacks) {
      callback(message);
    }
    uint16_t val = decode_u16(std::span(message).subspan<3, 2>());

    std::cout << "Debug print: " << std::dec << (uint)message[2] << " " << std::dec
              << (uint16_t)(val) << std::endl;
    // std::cout << "debug len:" << std::dec << (uint)message.size() <<
    // std::endl; for(auto i = 0; i < message.size(); i++) {
    //   std::cout << "debug " << i << ":" << std::dec << (uint)message[i] <<
    //   std::endl;
    // }
  } break;
  case TMX::MESSAGE_IN_TYPE::SERIAL_LOOP_BACK_REPORT:
    std::cout << "Serial loopback not implemented" << std::endl;
    break;
  case TMX::MESSAGE_IN_TYPE::DHT_REPORT: {
    for (const auto &callback : this->dht_callbacks) {
      callback(message);
    }
  } break;
  case TMX::MESSAGE_IN_TYPE::SPI_REPORT: {
    for (const auto &callback : this->spi_callbacks) {
      callback(message);
    }
  } break;
  case TMX::MESSAGE_IN_TYPE::SENSOR_REPORT: {
    for (const auto &callback : this->sensor_callbacks) {
      callback(message);
    }
  } break;
  case TMX::MESSAGE_IN_TYPE::MODULE_REPORT: {
    for (const auto &callback : this->module_callbacks) {
      callback(message);
    }
  } break;
  case TMX::MESSAGE_IN_TYPE::GET_ID_REPORT: {
    std::cout << "Get id not implemented" << std::endl;
    std::cout << "ID = " << std::hex << (uint)message[2] << std::endl;
  } break;
  case TMX::MESSAGE_IN_TYPE::SET_ID_REPORT: {
    std::cout << "Set id not implemented" << std::endl;
    std::cout << "ID = " << std::hex << (uint)message[2] << std::endl;
  } break;
  default:
    break;
  }
}

void TMX::sendPing(uint8_t num) {
  auto message = std::vector<uint8_t>{TMX::MESSAGE_TYPE::PING, (uint8_t)num};
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
}
/**
 * Send a message with some type. Length added automatically.
 */
void TMX::sendMessage(TMX::MESSAGE_TYPE type, const std::vector<uint8_t> &message) {
  std::vector<char> charMessage(message.begin(), message.end());
  charMessage.insert(charMessage.begin(), {(char)(charMessage.size() + 1), (char)type});

#ifdef TMX_TX_DEBUG
  std::cout << "T charMessage = ";
  for (auto i : charMessage) {
    std::cout << std::hex << (uint)(i & 0xFF) << " ";
  }
  std::cout << std::endl;
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
    if (pin < 26 || pin > 30) { // only pins 26-30 are analog
      return;
    }
    pin -= 26;
    message = std::vector<uint8_t>{pin, mode};
    // message.reserve(message.size() + sizeof(uint16_t) + sizeof(reporting));

    append_range(message, encode_u16(analog_differential));
    message.push_back((uint8_t)reporting);

    break;
  default:
    break;
  }
  assert(message.size() >= 2);
  this->sendMessage(TMX::MESSAGE_TYPE::SET_PIN_MODE, message);
}

void TMX::digitalWrite(uint8_t pin, bool value) {
  std::vector<uint8_t> message = {pin, value};
  this->sendMessage(TMX::MESSAGE_TYPE::DIGITAL_WRITE, message);
}
void TMX::pwmWrite(uint8_t pin, uint16_t value) {
  std::vector<uint8_t> message = {pin};
  append_range(message, encode_u16(value));
  this->sendMessage(TMX::MESSAGE_TYPE::PWM_WRITE, message);
}
void TMX::add_callback(MESSAGE_IN_TYPE type,
                       std::function<void(const std::vector<uint8_t> &)> callback) {
  switch (type) {
  case TMX::MESSAGE_IN_TYPE::PONG_REPORT:
    this->ping_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::FIRMWARE_REPORT:
    this->firmware_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::REPORT_PICO_UNIQUE_ID:
    this->pico_unique_id_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::SERIAL_LOOP_BACK_REPORT:
    this->serial_loop_back_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::DIGITAL_REPORT:
    this->digital_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::ANALOG_REPORT:
    this->analog_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::I2C_WRITE_REPORT:
    this->i2c_write_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::I2C_READ_FAILED:
    this->i2c_read_failed_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::I2C_READ_REPORT:
    this->i2c_read_failed_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::SONAR_DISTANCE:
    this->sonar_distance_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::DHT_REPORT:
    this->dht_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::SPI_REPORT:
    this->spi_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::ENCODER_REPORT:
    this->encoder_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::DEBUG_PRINT:
    this->debug_print_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::SENSOR_REPORT:
    this->sensor_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::MODULE_REPORT:
    this->module_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::SERVO_UNAVAILABLE:
    this->servo_unavailable_callbacks.push_back(callback);
    break;
  case TMX::MESSAGE_IN_TYPE::GET_ID_REPORT:
  case TMX::MESSAGE_IN_TYPE::SET_ID_REPORT:
    std::cout << "Get id and set id not implemented" << std::endl;
    break;
  default:
    break;
  }
}

void TMX::add_digital_callback(uint8_t pin, std::function<void(uint8_t, uint8_t)> callback) {
  this->digital_callbacks_pin.push_back({pin, callback});
}

void TMX::add_analog_callback(uint8_t pin, std::function<void(uint8_t, uint16_t)> callback) {
  // FIXME: This is Pico specific code
  if (pin < 26 || pin > 30) { // only pins 26-30 are analog
    return;
  }
  this->analog_callbacks_pin.push_back({pin - 26, callback});
  std::cout << "analog_callbacks_pin size = " << this->analog_callbacks_pin.size() << std::endl;
}

void TMX::attach_encoder(uint8_t pin_A, uint8_t pin_B,
                         std::function<void(uint8_t, int8_t)> callback) {
  this->encoder_callbacks_pin.push_back({pin_A, callback});
  uint8_t type = 2;                  // 'default' on quadrature encoder
  if (pin_B == 0xff || pin_B == 0) { // if pin_B is not set, go back to single
    pin_B = 0;
    type = 1;
  }
  this->sendMessage(TMX::MESSAGE_TYPE::ENCODER_NEW, {type, pin_A, pin_B});
}

void TMX::attach_sonar(uint8_t trigger, uint8_t echo,
                       std::function<void(uint8_t, uint16_t)> callback) {
  this->sonar_callbacks_pin.push_back({trigger, callback});
  this->sendMessage(TMX::MESSAGE_TYPE::SONAR_NEW, {trigger, echo});
}

void TMX::attach_servo(uint8_t pin, uint16_t min_pulse, uint16_t max_pulse) {
  // Arduino expects 5 bytes [pin, min_h, min_l, max_h, max_l]
  std::vector<uint8_t> msg = {pin};
  // msg.reserve(5);

  append_range(msg, encode_u16(min_pulse));
  append_range(msg, encode_u16(max_pulse));

  this->sendMessage(TMX::MESSAGE_TYPE::SERVO_ATTACH, msg);
}

void TMX::write_servo(uint8_t pin, uint16_t duty_cycle) {
  std::vector<uint8_t> msg = {pin};
  // msg.reserve(3);

  append_range(msg, encode_u16(duty_cycle));

  this->sendMessage(TMX::MESSAGE_TYPE::SERVO_WRITE, msg);
}

void TMX::detach_servo(uint8_t pin) { this->sendMessage(TMX::MESSAGE_TYPE::SERVO_DETACH, {pin}); }

void TMX::setScanDelay(uint8_t delay) {
  if (delay < 2) {
    delay = 2;
  }
  this->sendMessage(TMX::MESSAGE_TYPE::SET_SCAN_DELAY, {delay});
}

void TMX::stop() {
  // this->sendMessage(TMX::MESSAGE_TYPE::STOP, {});
  this->is_stopped = true;
  if (this->ping_thread.joinable() && std::this_thread::get_id() != this->ping_thread.get_id())
    this->ping_thread.join();
  this->stop_func = []() {};

  this->parsePool.stop();
  this->parsePool.join();

  if (!this->serial) {
    return;
  }

  if (this->serial->isOpen()) {
    this->sendMessage(TMX::MESSAGE_TYPE::RESET_BOARD, {});
    this->serial->close();
  }
}

bool TMX::setI2CPins(uint8_t sda, uint8_t scl, uint8_t port) {
  if (sda == 0 || scl == 0 || sda == scl) {
    return false;
  }
  static bool initialized_ports[2] = {false, false}; // 2 ports for now
  if (initialized_ports[port]) {
    return false;
  }
  initialized_ports[port] = true;
  // TODO: add a check for pins, store some map of current pins
  this->sendMessage(TMX::MESSAGE_TYPE::I2C_BEGIN, {port, sda, scl});
  return true;
}

std::pair<bool, std::vector<uint8_t>> TMX::parse_buffer_for_message(std::vector<uint8_t> &buffer,
                                                                    uint8_t wanted_len,
                                                                    uint8_t wanted_type) {
  if (buffer.size() < wanted_len) {
    return {false, {}};
  }

  // expected message: {wanted_len, type, ...}
  std::cout << "buffer size: " << buffer.size() << std::endl;
  while (buffer.size() >= wanted_len) {
    std::cout << "buffer size: " << buffer.size() << std::endl;
    auto len = buffer[0];
    std::cout << "len: " << (int)len << std::endl;
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
      return {true, {buffer.begin() + 1, buffer.begin() + 1 + wanted_len}};
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
        std::cout << "check port len: " << len << std::endl;
        buffer.insert(buffer.end(), data, data + len);
      });
      buffer.clear();
      serial->write({0, 0, 0, 0, 0, 0, 0, 1,
                     MESSAGE_TYPE::FIRMWARE_VERSION}); // send a get fw version message
      std::this_thread::sleep_for(
          std::chrono::milliseconds(100)); // pico should respond within 100ms
      serial->close();
      auto out = TMX::parse_buffer_for_message(buffer, 4, MESSAGE_IN_TYPE::FIRMWARE_REPORT);
      if (out.first) {
        return true;
      } else {
        return false;
      }
    } catch (std::exception &e) {
      std::cout << "Exception" << e.what() << std::endl;
      return false;
    }
  });

  std::future_status status;

  status = future.wait_for(std::chrono::milliseconds(200));

  if (status == std::future_status::timeout) {
    // verySlow() is not complete.
    return false;
  } else if (status == std::future_status::ready) {
    // verySlow() is complete.
    // Get result from future (if there's a need)
    auto ret = future.get();
    return ret;
  }

  return false;
}

const std::vector<TMX::serial_port> TMX::accepted_ports = {
    {"", 0x1a86, 0x7523}, // CH340
    {"", 0x2E8A, 0x000A}, // RP2040
};

#include <boost/format.hpp> // std::format not yet supported
#include <filesystem>
#include <iostream>
#include <string>
#include <tmx_cpp/tmx_util.hpp>
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

          auto out = exec((boost::format("udevadm info --name=%s | grep 'ID_VENDOR_ID'") %
                           canonical_path.generic_string())
                              .str());

          out = out.substr(out.find('=') + 1);  // ID_VENDOR_ID=0403, only get the 0403
          sp.pid = std::stoi(out, nullptr, 16); // convert to int
          out = exec((boost::format("udevadm info --name=%s | grep ID_MODEL_ID=") %
                      canonical_path.generic_string())
                         .str());
          out = out.substr(out.find('=') + 1);
          sp.vid = std::stoi(out, nullptr, 16);
          std::cout << sp.port_name << " " << sp.vid << ":" << sp.pid << std::endl;
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
  serial->write({1, MESSAGE_TYPE::GET_ID});                     // send a get id message
  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // pico should respond within 100ms
  serial->close();
  auto out = TMX::parse_buffer_for_message(buffer, 3, MESSAGE_IN_TYPE::GET_ID_REPORT);
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
  serial->write({2, MESSAGE_TYPE::SET_ID, (char)id});           // send a set id message
  std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // pico should respond within 100ms
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
  auto out = TMX::parse_buffer_for_message(buffer, 3, MESSAGE_IN_TYPE::SET_ID_REPORT);
  // expected message: {2, MESSAGE_IN_TYPE::SET_ID_REPORT, id}
  return out.first && out.second[1] == id;
}

void TMX::ping_task() {
  uint8_t num = 0;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (!this->is_stopped) {
    num++;
    this->sendPing(num);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    if ((num - this->last_ping) > 2) {
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
    std::cout << "Magic changed" << (int)this->magic << "-> " << (int)message[3] << std::endl;
    this->stop_func();
  }
  this->last_ping = message[2];
}
