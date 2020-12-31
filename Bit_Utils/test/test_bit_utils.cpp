#include <gtest/gtest.h>
#include <bitset>
#include <iostream>
#include "Bit_Utils/bit_utils.h"

using namespace bit_utils;

TEST(TestEndian, TestEndian)
{
    ASSERT_TRUE(bit_utils::little_endian());
}

TEST(TestEndian, TestFlipEndian)
{
    std::bitset<2> bits;
    bits[0] = 1;
    bits[1] = 0;
    std::bitset<2> flipped_bits = bit_utils::flip_endian(bits);
    ASSERT_EQ(flipped_bits[0], 0);
    ASSERT_EQ(flipped_bits[1], 1);
}

TEST(GPS, TLM_Word)
{
    uint32_t tlm_word = 583029304;
    std::bitset<30> bits(tlm_word);
    std::cout<<bits.to_string()<<std::endl;
    ASSERT_EQ(get_bit_msb(bits, 1), 1);
    ASSERT_EQ(get_bit_msb(bits, 2), 0);
}

TEST(GPS, HOV_Word)
{
    uint32_t hov_word = 709937912;
    std::bitset<30> bits(hov_word);
    std::cout<<bits.to_string()<<std::endl;
    ASSERT_EQ(get_bit_msb(bits, 29), 0);
    ASSERT_EQ(get_bit_msb(bits, 30), 0);
}