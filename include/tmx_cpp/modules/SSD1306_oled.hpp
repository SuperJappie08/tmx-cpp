#pragma once
#include <future>
#include <string>

#include "tmx_cpp/modules/Module_t.hpp"

namespace tmx_cpp
{

class SSD1306_module : public Module_type
{
public:
  std::function<void(std::vector<uint8_t>)> send_module;
  SSD1306_module(
    uint8_t i2c_port, uint8_t address = 0x3C, uint8_t width = 128, uint8_t height = 64);

  enum OLED_MSG_TYPES : uint8_t
  {
    TEXT = 0,
    TEXT_DONE = 1,
    BINARY = 2,
    BINARY_DONE = 3
  };

  virtual std::vector<uint8_t> init_data() override;
  virtual void data_callback(std::vector<uint8_t> data) override;
  // FIXME: Maybe give this a default implementation, since all implementations are the same.
  virtual void attach_send_module(std::function<void(std::vector<uint8_t>)> send_module) override;

  bool send_text(std::string text);
  bool send_image(uint8_t width, uint8_t height, uint8_t img_buffer[]);

private:
  uint8_t i2c_port;
  uint8_t address;

  uint8_t width_;
  uint8_t height_;

  std::promise<std::vector<uint8_t>> text_promise;
  std::promise<std::vector<uint8_t>> binary_promise;
};

}  // namespace tmx_cpp