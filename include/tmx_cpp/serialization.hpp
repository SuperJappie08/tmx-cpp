#pragma once
#include <cstdint>
#include <span>
#include <vector>

namespace tmx_cpp {

/// @brief Append the `extra` vector to the data vector.
/// @param data The vector to append to.
/// @param extra The vector which will be appended
/// @return An iterator that points to the inserted data.
inline std::vector<uint8_t>::iterator append_range(std::vector<uint8_t> &data,
                                                   std::vector<uint8_t> extra) {
  return data.insert(data.end(), extra.begin(), extra.end());
}

// Encode
std::vector<uint8_t> encode_u16(const uint16_t &value);
std::vector<uint8_t> encode_u32(const uint32_t &value);
std::vector<uint8_t> encode_u64(const uint64_t &value);

std::vector<uint8_t> encode_i16(const int16_t &value);
std::vector<uint8_t> encode_i32(const int32_t &value);
std::vector<uint8_t> encode_i64(const int64_t &value);

std::vector<uint8_t> encode_float(const float &value);

// TODO: Maybe put implementation here. So things can be inlined

// Decode
uint16_t decode_u16(const std::span<const uint8_t, 2> &data);
uint32_t decode_u32(const std::span<const uint8_t, 4> &data);
uint64_t decode_u64(const std::span<const uint8_t, 8> &data);

int16_t decode_i16(const std::span<const uint8_t, 2> &data);
int32_t decode_i32(const std::span<const uint8_t, 4> &data);
int64_t decode_i64(const std::span<const uint8_t, 8> &data);

float decode_float(const std::span<const uint8_t, 4> &data);

} // namespace tmx_cpp
