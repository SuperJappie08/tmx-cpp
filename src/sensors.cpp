#include "sensors.hpp"

Sensors::Sensors(TMX *tmx) {
  this->tmx = tmx;
  tmx->add_callback(TMX::MESSAGE_IN_TYPE::SENSOR_REPORT,
                    std::bind(&Sensors::callback, this, std::placeholders::_1));
}

void Sensors::add_sensor(SENSOR_TYPE type, std::vector<uint8_t> data,
                         std::function<void(std::vector<uint8_t>)> callback) {
                          // todo: send data to pico
  sensors.push_back(std::make_pair(type, callback));
}

void Sensors::add_adxl345(uint8_t i2c_port,
                          std::function<void(std::vector<uint8_t>)> callback) {
  add_sensor(SENSOR_TYPE::ADXL345, {i2c_port},
             [&callback](std::vector<uint8_t> data) {
               // TODO: add adxl parsing
               callback(data);
             });
}

void Sensors::add_veml6040(uint8_t i2c_port,
                           std::function<void(std::vector<uint8_t>)> callback) {
  add_sensor(SENSOR_TYPE::VEML6040, {i2c_port},
             [&callback](std::vector<uint8_t> data) {
               // TODO: add veml parsing
               callback(data);
             });
}

void Sensors::callback(std::vector<uint8_t> data) {
  uint8_t sensor_num = data[1]; // TODO: check
  this->sensors[sensor_num].second(data);
}