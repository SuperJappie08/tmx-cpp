#include <cassert>
#include <iostream>
#include <span>

#include <tmx_cpp/serialization.hpp>

int main() {
  // TEST uint16_t
  {
    uint16_t num = 0;
    do {
      auto val = tmx_cpp::encode_u16(num);
      auto sup_val = tmx_cpp::decode_u16(std::span(val).first<sizeof(uint16_t)>());
      std::cout << "\rU16: " << num << "/" << UINT16_MAX << std::flush;
      assert(sup_val == num);
      num++;
    } while (num != 0);
  }

  {
    uint32_t num = 0;
    do {
      auto val = tmx_cpp::encode_u32(num);
      auto sup_val = tmx_cpp::decode_u32(std::span(val).first<sizeof(uint32_t)>());
      std::cout << "\rU32: " << num << "/" << UINT32_MAX << std::flush;
      assert(sup_val == num);
      if (num == 0)
        num++;
      else
        num = num << 1;
    } while (num != 0);
  }

  {
    uint64_t num = 0;
    do {
      auto val = tmx_cpp::encode_u64(num);
      auto sup_val = tmx_cpp::decode_u64(std::span(val).first<sizeof(uint64_t)>());
      std::cout << "\rU64: " << num << "/" << UINT64_MAX << std::flush;
      assert(sup_val == num);
      if (num == 0)
        num++;
      else
        num = num << 1;
    } while (num != 0);
  }

  std::cout << "\r >>> DONE ";
  for (int idx = 0; idx < (64 - 11); idx++)
    std::cout << " ";
  std::cout << std::endl;
  return 0;
}