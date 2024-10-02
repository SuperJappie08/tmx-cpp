#include <bit>
#include <cstdint>

#include <tmx_cpp/tmx_util.hpp>

namespace tmx_cpp
{

std::string exec(const std::string & cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);

  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

float decode_float(const std::vector<uint8_t> data)
{
  return std::bit_cast<float>(decode_u32(data));
}

uint32_t decode_u32(const std::vector<uint8_t> data)
{
  return (((uint32_t)data[3]) << 24) | (((uint32_t)data[2]) << 16) | (((uint32_t)data[1]) << 8) |
         ((uint32_t)data[0]);
}

}  // namespace tmx_cpp
