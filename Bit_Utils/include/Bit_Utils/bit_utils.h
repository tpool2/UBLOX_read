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
        result[bit_index] = bits[num_bits-bit_index-1];
    }
    return result;
}

template<size_t start_pos, size_t end_pos, size_t num_bits> std::bitset<end_pos-start_pos> subset(const std::bitset<num_bits> &bits)
{
    std::bitset<end_pos-start_pos> sub_bits;
    for(int index = 0; index < sub_bits.size(); ++index)
    {
        sub_bits[index] = bits[index+start_pos];
    }
    return sub_bits;
}

template <class T> T get_bits_msb(const T number, int start, int stop)
{
    int bit_quantity = stop - start;
    size_t bit_quantity_T = sizeof(T)*8;
    int shift_factor = bit_quantity_T-start-bit_quantity;
    // std::cout<<"Bit Quantity: "<<bit_quantity<<std::endl;
    // std::cout<<"bit_quantity_T: "<<bit_quantity_T<<std::endl;
    // std::cout<<"Shift Factor: "<<shift_factor<<std::endl;
    return T((number & ((1<<bit_quantity)-1)<<shift_factor)>>shift_factor);
}

template <class T> uint8_t get_bits_straddle(const T numbers, int start, int stop)
{
    size_t bit_quantity_T = sizeof(T)*8;
    int start_word = start / bit_quantity_T;
    int start_bit = start % bit_quantity_T;
    int bit_quantity = stop - start;
    uint8_t answer = 0;
    for(int bit_count = 0; bit_count < bit_quantity; ++bit_count)
    {
        answer = (answer << 1) | get_bits_msb(numbers[(start+bit_count)/bit_quantity_T], (start+bit_count)%bit_quantity_T, ((start+bit_count)%bit_quantity_T)+1);
    }
    return answer;
}
}

#endif