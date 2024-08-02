#pragma once
#include "modules/Module_types.hpp"
#include <functional>
#include <stdint.h>
#include <vector>
class Module_type {
public:
  virtual std::vector<uint8_t> init_data() = 0;
  virtual void data_callback(std::vector<uint8_t> data) = 0;
  MODULE_TYPE type;
  virtual void
  attach_send_module(std::function<void(std::vector<uint8_t>)> send_module) = 0;
  virtual ~Module_type() = default;
};