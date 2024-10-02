#pragma once

#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace tmx_cpp {

// same as in mirte-telemetrix-cpp/src/util.cpp
std::string exec(const std::string & cmd);

float decode_float(const std::vector<uint8_t> data);
uint32_t decode_u32(const std::vector<uint8_t> data);

}  // namespace tmx_cpp