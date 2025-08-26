#pragma once

#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "tmx_cpp/compatibility.hpp"

namespace tmx_cpp {

#if !defined(TMX_CPP_WINDOWS)
// same as in mirte-telemetrix-cpp/src/util.cpp
std::string exec(const std::string &cmd);
#endif

} // namespace tmx_cpp
