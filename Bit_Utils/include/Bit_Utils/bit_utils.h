#ifndef BIT_UTILS
#define BIT_UTILS
#include <bitset>

namespace bit_utils
{

bool little_endian();
template<size_t num_bits> bool get_bit_msb(const std::bitset<num_bits> &bits, size_t pos)
{
    return bits[num_bits-pos];
}

}

#endif