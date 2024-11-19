#include <chrono>
#include <iostream>

#include "tmx_cpp/modules/SSD1306_oled.hpp"
#include "tmx_cpp/types.hpp"

using namespace tmx_cpp;

SSD1306_module::SSD1306_module(uint8_t i2c_port, uint8_t address, uint8_t width, uint8_t height)
    : i2c_port(i2c_port), address(address), width_(width), height_(height) {
  type = MODULE_TYPE::TMX_SSD1306;
}

std::vector<uint8_t> SSD1306_module::init_data() { return {this->i2c_port, this->address}; }

void SSD1306_module::data_callback(std::vector<uint8_t> data) {
  // std::cout << "R | OLED chardata = ";
  // for (auto i : data) {
  //   std::cout << std::hex << (uint)(i & 0xFF) << " ";
  // }
  // std::cout << std::endl;

  // std::cout << "OLED | MSG LEN : " << data.size() << std::endl;
  // std::cout << (int)data[1] << std::endl;

  // data := {TYPE, LEN, x i2c_sdk_call_return_value, display enabled}
  auto msg_type = data[0];

  // TODO: DO SOMETHING WITH THE DATA

  // TODO: Maybe clear other promises?

  // Recieves a message if the writing is finished.
  switch (msg_type) {
  case TEXT_DONE:
    text_promise.set_value(std::vector<uint8_t>(data.cbegin() + 1, data.cend()));
    return;
  case BINARY_DONE:
    // LEN == 1
    binary_promise.set_value(std::vector<uint8_t>(data.cbegin() + 1, data.cend()));
    return;
  default:
    // Other cases shouldn't happen. Unless display is disabled
    std::cerr << "An OLED Error message was Recieved: ";
    for (auto i : data)
      std::cerr << std::hex << (uint)(i & 0xFF) << " ";
    std::cerr << std::endl;
    std::cerr << "The OLED probably gets deactivated now." << std::endl;
    break;
  }

  return;
}

bool SSD1306_module::send_text(std::string text, std::chrono::milliseconds timeout) {
  auto future_response = text_promise.get_future();

  // Truncate to char the character limit.
  auto truncated_text = text.substr(0, std::min((size_t)character_limit, text.length()));

  auto max_msg_len = 30 - 4;

  for (int idx = 0; idx < truncated_text.length(); idx += max_msg_len) {
    auto len = (idx + max_msg_len <= truncated_text.length())
                   ? max_msg_len
                   : (truncated_text.length() % max_msg_len);
    auto text_segment = truncated_text.substr(idx, len);

    std::vector<uint8_t> data = {TEXT, (uint8_t)len};
    data.insert(data.end(), text_segment.begin(), text_segment.end());
    send_module(data);
  }
  send_module({TEXT_DONE});

  if (future_response.wait_for(timeout) != std::future_status::ready) {
    std::cerr << "Text timeout error" << std::endl;
    text_promise = std::promise<std::vector<uint8_t>>();
    return false;
  }
  auto response = future_response.get();
  text_promise = std::promise<std::vector<uint8_t>>();

  auto supposed_text_length = response[0];
  if (supposed_text_length > character_limit) {
    // TODO: Improve Warning
    std::cerr << "Text is too long" << std::endl;
    return false;
  } else if (supposed_text_length != (uint8_t)truncated_text.length()) {
    // TODO: Improve Warning
    std::cerr << "Text wrong length" << std::endl;
    // TODO: This was not here in the python version, should it be there?
    // return false;
  }
  return true;
}

// TODO: IMPROVE DOCS
/// @brief Send an image to the SSD1306 OLED Screen
/// @param img_buffer The image buffer with MONO8 encoding
/// @return true if succesfull
bool SSD1306_module::send_image(uint8_t width, uint8_t height, uint8_t img_buffer[],
                                std::chrono::milliseconds timeout) {
  // TODO: Timeout

  if (width != width_ || height != height_) {
    std::cerr << "Image must be same dimensions as display (" << width_ << "x" << height_ << ")"
              << std::endl;
    return false;
  }

  auto future_response = binary_promise.get_future();

  std::vector<uint8_t> compressed_buffer;

  for (auto height_block = 0; height_block < height / 8; height_block++) {
    for (auto x = 0; x < width; x++) {
      uint8_t byt = 0;
      for (uint8_t bit = 0; bit < 8; bit++) {
        byt = byt << 1;
        byt |= (img_buffer[x + (7 - bit) * width + height_block * width * 8] > 126) ? 1 : 0;
      }
      compressed_buffer.push_back(byt);
    }
  }

  auto max_msg_len = 16;

  for (uint8_t idx = 0; idx < compressed_buffer.size() / max_msg_len; idx++) {
    std::vector<uint8_t> data = {BINARY, idx};
    data.insert(data.end(), compressed_buffer.cbegin() + idx * max_msg_len,
                compressed_buffer.cbegin() + (idx + 1) * max_msg_len);
    send_module(data);
  }
  send_module({BINARY_DONE});

  if (future_response.wait_for(timeout) != std::future_status::ready) {
    std::cerr << "Image timeout error" << std::endl;
    binary_promise = std::promise<std::vector<uint8_t>>();
    return false;
  }
  auto response = future_response.get();
  binary_promise = std::promise<std::vector<uint8_t>>();

  auto acknowledgement = response[0];
  if (acknowledgement > 200) {
    // TODO: Improve Warning
    std::cerr << "Write failed oled send_write img" << std::endl;
    return false;
  } else if (acknowledgement != (uint8_t)1) {
    // TODO: Improve Warning
    std::cerr << "Error acknowledged" << std::endl;
    // TODO: This was not here in the python version, should it be there?
    // return false;
  }
  return true;
}

void SSD1306_module::attach_send_module(std::function<void(std::vector<uint8_t>)> send_module) {
  this->send_module = send_module;
}