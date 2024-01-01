// typedef std::vector<uint8_t> byte_buffer;

// template <std::size_t N>
// void append_fixed_width(byte_buffer& buf, uintmax_t val) {
//     int shift = ((N - 1) * 8);
//     while (shift >= 0) {
//         uintmax_t mask = (0xff << shift);
//         buf.push_back(uint8_t((val & mask) >> shift));
//         shift -= 8;
//     }
// }

// template <typename IntType>
// void append_bytes(byte_buffer& buf, IntType val) {
//     append_fixed_width<sizeof(IntType)>(buf, uintmax_t(val));
// }