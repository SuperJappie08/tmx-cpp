#pragma once
#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <vector>
#include <modules/PCA9685.hpp>
#include <memory>
class TMX;
class Modules {
public:
  enum MODULE_TYPE : uint8_t;
  int add_module(MODULE_TYPE type, std::vector<uint8_t> data,
                  std::function<void(std::vector<uint8_t>)> callback);
  enum MODULE_TYPE : uint8_t {
    PCA9685 = 0,
        HIWONDERSERVO = 1,
        SHUTDOWN_RELAY = 2,
        TMX_SSD1306 = 3,
  };
  std::vector<std::pair<MODULE_TYPE, std::function<void(std::vector<uint8_t>)>>>
      modules;
    TMX *tmx;

  Modules(TMX *tmx);
  void callback(std::vector<uint8_t> data);

        bool send_module(uint8_t module_num, std::vector<uint8_t> data);

        std::shared_ptr<PCA9685_module> add_pca9685(uint8_t i2c_port, uint8_t address = 0x40, int frequency = 200);


private:
};
void empty_callback (std::vector<uint8_t> data);

#include "tmx.hpp" // fix for circular dependency