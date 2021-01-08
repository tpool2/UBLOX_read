#ifndef BIT_UTILS
#define BIT_UTILS
#include <bitset>
#include <iostream>
#include <type_traits>

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

template <class T, class R> T get_bits(const R* buffer, int position, int length = sizeof(T))
{
    size_t T_size = sizeof(T)*8;
    size_t R_size = sizeof(R)*8;

    T bits = 0;
    
    for(int bit_index = 0; bit_index < length; ++bit_index)
    {
        int word_index = (position+bit_index)/R_size;
        int bit_in_word_index = (position+bit_index)%R_size;
        int shift_factor = R_size - bit_in_word_index - 1;
        ulong bit_value = ((buffer[word_index] & (1UL<<shift_factor))>>shift_factor);
        bits |= (bit_value & 1UL) << (length-bit_index-1);
        // std::cout<< bit_index<<" " << bit_value << " " <<static_cast<uint16_t>(bits)<<std::endl;
    }

    // std::cout<<"T_size: "<<T_size << " length: " << length << std::endl; 
    if(std::is_signed<T>::value && (bits & (1UL<<(length-1))) && T_size > length)
    {
        // std::cout<<"Needs conversion"<<std::endl;
        bits |= (((1UL<<(T_size-length))-1)<<length);
    }
    
    return bits;
}
}

#endif