#include "tmx.hpp"
#include <algorithm>
#include <cassert>
#include <iostream>
// #include <ranges>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <utility>
TMX::TMX(std::string port) : parsePool(10), sensors(this) {
  this->serial = new CallbackAsyncSerial(port, 115200);
}

TMX::~TMX() {}

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
void TMX::parseOne(std::vector<uint8_t> &message) {
  boost::asio::post(this->parsePool,
                    std::bind(&TMX::parseOne_task, this, message));
}

void TMX::parseOne_task(std::vector<uint8_t> &message) {
  // Note:: this runs on a different thread than any other things.
  // Makes it possible to have longer running callbacks without interfering with
  // other callbacks and reading in data.

  auto type = (TMX::MESSAGE_IN_TYPE)buffer[1];
  switch (type) {
  case TMX::MESSAGE_IN_TYPE::PONG_REPORT: {
    for (const auto &callback : this->ping_callbacks) {
      callback(message);
    }
  } break;
  case TMX::MESSAGE_IN_TYPE::FIRMWARE_REPORT:
    std::cout << "firmware version: " << (int)message[2] << '.'
              << (int)message[3] << std::endl;
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
  }
  case TMX::MESSAGE_IN_TYPE::ANALOG_REPORT: {
    auto pin = message[2];
    auto value = (message[3] << 8) | message[4];
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
    auto value =
        (message[3] << 8) | message[4]; // TODO: check if this is correct
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
    auto pin = message[2];
    auto value =
        (message[3] << 8) | message[4]; // TODO: check if this is correct
    for (const auto &callback : this->encoder_callbacks_pin) {
      if (callback.first == pin) {
        callback.second(pin, value);
      }
    }
    for (const auto &callback : this->encoder_callbacks) {
      callback(message);
    }
  }
  case TMX::MESSAGE_IN_TYPE::DEBUG_PRINT: {
    for (const auto &callback : this->debug_print_callbacks) {
      callback(message);
    }
    std::cout << "Debug print: " << std::hex << (uint)message[1] << " "
              << std::hex << (uint)message[2] << std::endl;
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

  default:
    break;
  }
}

void TMX::sendPing() {

  static auto i = 0;
  auto message = std::vector<uint8_t>{TMX::MESSAGE_TYPE::PING, (uint8_t)i};
  i++;
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
void TMX::sendMessage(TMX::MESSAGE_TYPE type,
                      const std::vector<uint8_t> &message) {
  std::vector<char> charMessage(message.begin(), message.end());
  charMessage.insert(charMessage.begin(),
                     {(char)(charMessage.size() + 1), (char)type});
  std::cout << "charMessage = ";
  for (auto i : charMessage) {
    std::cout << std::hex << (int)i << " ";
  }
  std::cout << std::endl;
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
    message = std::vector<uint8_t>{
        pin, mode, (uint8_t)((analog_differential >> 8) & 0xff),
        (uint8_t)(analog_differential & 0xff), reporting};
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

void TMX::add_callback(MESSAGE_IN_TYPE type,
                       std::function<void(std::vector<uint8_t>)> callback) {
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
  this->analog_callbacks_pin.push_back({pin, callback});
}

void TMX::attach_encoder(uint8_t pin_A, uint8_t pin_B,
                         std::function<void(uint8_t, int8_t)> callback) {
  this->encoder_callbacks_pin.push_back({pin_A, callback});
  this->sendMessage(TMX::MESSAGE_TYPE::ENCODER_NEW, {pin_A, pin_B});
}

void TMX::attach_sonar(uint8_t trigger, uint8_t echo,
                       std::function<void(uint8_t, uint16_t)> callback) {
  this->sonar_callbacks_pin.push_back({trigger, callback});
  this->sendMessage(TMX::MESSAGE_TYPE::SONAR_NEW, {trigger, echo});
}