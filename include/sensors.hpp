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
class TMX;
class Sensors {
public:
  enum SENSOR_TYPE : uint8_t;
  void add_sensor(SENSOR_TYPE type, std::vector<uint8_t> data,
                  std::function<void(std::vector<uint8_t>)> callback);
  enum SENSOR_TYPE : uint8_t {
    ADXL345 = 0x01,
    VEML6040 = 0x02,
  };
  std::vector<std::pair<SENSOR_TYPE, std::function<void(std::vector<uint8_t>)>>>
      sensors;
  void add_adxl345(uint8_t i2c_port,
                   std::function<void(std::vector<uint8_t>)>
                       callback); // todo: change to adxl data function
  void add_veml6040(uint8_t i2c_port,
                    std::function<void(std::vector<uint8_t>)> callback);
  TMX *tmx;
  Sensors(TMX *tmx);
  void callback(std::vector<uint8_t> data);

private:
};

#include "tmx.hpp" // fix for circular dependency