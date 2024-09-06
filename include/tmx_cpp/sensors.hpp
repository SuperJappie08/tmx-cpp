#pragma once
#include "tmx_cpp/sensors/Sensor_t.hpp"
#include "tmx_cpp/sensors/Sensor_types.hpp"
#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <memory>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <vector>
class TMX;
class Sensors {
public:
  size_t add_sensor(uint8_t sens_num, SENSOR_TYPE type, std::vector<uint8_t> data,
                 std::function<void(std::vector<uint8_t>)> callback);
  std::vector<std::pair<SENSOR_TYPE, std::function<void(std::vector<uint8_t>)>>>
      sensors;
  // void add_adxl345(uint8_t i2c_port,
  //                  std::function<void(std::vector<uint8_t>)>
  //                      callback); // todo: change to adxl data function
  // void add_veml6040(uint8_t i2c_port,
  //                   std::function<void(std::vector<uint8_t>)> callback);
  std::shared_ptr<TMX> tmx;
  Sensors(std::shared_ptr<TMX> tmx);
  void callback(std::vector<uint8_t> data);
  void add_sens(std::shared_ptr<Sensor_type> module);
  // void add_sensor(std::shared_ptr<Sensor_type> module);
private:
};

#include "tmx.hpp" // fix for circular dependency