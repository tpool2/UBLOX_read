#ifndef BIT_UTILS
#define BIT_UTILS
#include <bitset>
#include <iostream>

namespace bit_utils
{

bool little_endian();
template<size_t num_bits> bool get_bit_msb(const std::bitset<num_bits> &bits, size_t pos)
{
    return bits[num_bits-pos];
}

template<size_t num_bits> std::bitset<num_bits> flip_endian(const std::bitset<num_bits> &bits)
{
    std::bitset<num_bits> result;
    for(int bit_index = 0; bit_index < num_bits; ++bit_index)
    {
        // std::cout<<"Assigning "<<bit_index<< " the value " << bits[num_bits-bit_index-1] << " at " << num_bits-bit_index<<std::endl;
        result[bit_index] = bits[num_bits-bit_index-1];
    }
    return result;
}

}

#endif