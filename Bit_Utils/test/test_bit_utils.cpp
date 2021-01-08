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

TEST(TestEndian, TestFlipEndian2)
{
    std::bitset<4> bits("0111");
    ASSERT_EQ(bit_utils::flip_endian(bits), std::bitset<4>("1110"));
}

TEST(GPS, TLM_Word)
{
    uint32_t tlm_word = 583029304;
    std::bitset<30> bits(tlm_word);
    // std::cout<<bits.to_string()<<std::endl;
    ASSERT_EQ(get_bit_msb(bits, 1), 1);
    ASSERT_EQ(get_bit_msb(bits, 2), 0);
    ASSERT_EQ(get_bits<uint8_t>(&tlm_word, 2, 1), 1);
    ASSERT_EQ(get_bits<uint8_t>(&tlm_word, 2, 8), 139);
}

TEST(GPS, HOV_Word)
{
    uint32_t hov_word = 709937912;
    std::bitset<30> bits(hov_word);
    // std::cout<<bits.to_string()<<std::endl;
    ASSERT_EQ(get_bit_msb(bits, 29), 0);
    ASSERT_EQ(get_bit_msb(bits, 30), 0);
}

TEST(Subbits, S0111_01)
{
    std::bitset<4> bits;
    bits[1] = 1;
    bits[2] = 1;
    bits[3] = 1;

    std::bitset<2> answer;
    answer[0] = 0;
    answer[1] = 1;
    ASSERT_EQ( (subset<0,2>(bits)), answer);
}

TEST(Subbits, S0111_011)
{
    std::bitset<4> bits;
    bits[1] = 1;
    bits[2] = 1;
    bits[3] = 1;

    std::bitset<3> answer;
    answer[0] = 0;
    answer[1] = 1;
    answer[2] = 1;
    ASSERT_EQ( (subset<0,3>(bits)), answer);
}

TEST(BitsinBytes, byte1)
{
    uint8_t my_byte = 0b00001111;
    ASSERT_EQ((my_byte & 0x01), 0b1);
}

TEST(BitsinBytes, bit2)
{
    uint8_t my_byte = 0b00001111;
    ASSERT_EQ((my_byte & 0x02)>>1, 0b1);
}

TEST(BitsinBytes, bit3)
{
    uint8_t my_byte = 0b00001011;
    ASSERT_EQ((my_byte & (0x1<<2))>>2, 0b0);
    ASSERT_NE((my_byte & (0x1<<2))>>2, 0b1);
}

TEST(BitsinBytes, bits1and2)
{
    uint8_t my_byte = 0b00001011;
    ASSERT_EQ((my_byte & (0b11<<0))>>0, 0b11);
}

TEST(BitsinBytes, bits1and2and3and4)
{
    uint8_t my_byte = 0b00001011;
    ASSERT_EQ((my_byte & (0b1111<<0))>>0, 0b1011);
}

TEST(BitsinBytes, bits2and3and4)
{
    uint8_t my_byte = 0b00001011;
    ASSERT_EQ((my_byte & (0b111<<1))>>1, 0b101);
}

TEST(GetBitsMSB, Get_MSB_1)
{
    uint32_t number = 0xFF000000;
    ASSERT_EQ(get_bits_msb(number, 0, 1), 1);
}

TEST(GetBitsMSB, Get_LSB_0)
{
    uint32_t number = 0xFF000000;
    ASSERT_EQ(get_bits_msb(number, 31, 32), 0);
}

TEST(GetBitsMSB, GPSPreamble_0_1)
{
    uint8_t number = 139;
    ASSERT_EQ(get_bits_msb(number, 0, 1), 1);
}

TEST(GetBitsMSB, GPSPreamble_1_0)
{
    uint8_t number = 139;
    ASSERT_EQ(get_bits_msb(number, 1, 2), 0);
}

TEST(GetBitsMSB, GPSPreamble_2_0)
{
    uint8_t number = 139;
    ASSERT_EQ(get_bits_msb(number, 2, 3), 0);
}

TEST(GetBitsMSB, GPSPreamble_3_0)
{
    uint8_t number = 139;
    ASSERT_EQ(get_bits_msb(number, 3, 4), 0);
}

TEST(GetBitsMSB, GPSPreamble_4_1)
{
    uint8_t number = 139;
    ASSERT_EQ(get_bits_msb(number, 4, 5), 1);
}

TEST(GetBitsMSB, GPSPreamble_5_0)
{
    uint8_t number = 139;
    ASSERT_EQ(get_bits_msb(number, 5, 6), 0);
}

TEST(GetBitsMSB, GPSPreamble_6_1)
{
    uint8_t number = 139;
    ASSERT_EQ(get_bits_msb(number, 6, 7), 1);
}

TEST(GetBitsMSB, GPSPreamble_7_1)
{
    uint8_t number = 139;
    ASSERT_EQ(get_bits_msb(number, 7, 8), 1);
}

TEST(LeftShift, From128_To256)
{
    uint8_t number = 128;
    ASSERT_EQ(number<<1, 256);
}

TEST(GetBitsMSB, StraddleBytes)
{
    uint8_t numbers[2];
    numbers[0] = 0x01;
    numbers[1] = 0b10000000;
    int start_bit = 7;
    int num_bits = 2;

    uint8_t answer = 0;
    
    for(int bit_index = 0; bit_index < num_bits; ++bit_index)
    {
        int word_index = (start_bit+bit_index) / 8;
        int bit_in_word_index = (start_bit+bit_index) % 8;
        int shift_factor = 7 - bit_in_word_index;
        // std::cout << "Word: "<<word_index << " Bit in Word: " << bit_in_word_index;
        bool bit_value = ((numbers[word_index] & (1<<shift_factor))>>shift_factor);
        // std::cout << " Bit Value: " << bit_value;
        answer |= 1UL << (num_bits-bit_index-1);
        // std::cout << " Answer: " << (uint16_t)answer<< std::endl;
    }
    ASSERT_EQ(answer, 3);
}

TEST(GetBitsMSB, StraddleByte2)
{
    uint8_t numbers[2];
    numbers[0] = 0x01;
    numbers[1] = 0b11000000;
    int start_bit = 7;
    int num_bits = 3;

    uint8_t answer = get_bits<uint8_t>(numbers, start_bit, num_bits);
    ASSERT_EQ(answer, 7);
}

TEST(GetBitsMSB, StraddleByte3)
{
    uint8_t numbers[2];
    numbers[0] = 0x01;
    numbers[1] = 0b01000000;
    
    uint8_t answer = get_bits<uint8_t>(numbers, 7, 3);
    ASSERT_EQ(answer, 5);
}

TEST(Unsigned_to_Signed, From32769_to_32767)
{
    uint16_t number = 0b1000000000000001;
    ASSERT_EQ(static_cast<int16_t>(number), -32767);
}

TEST(Unsigned_to_Signed, From128_to_128)
{
    uint8_t number = 128;
    ASSERT_EQ(static_cast<int8_t>(number), -128);
}

TEST(IncreaseSignedTypeSize, FiveBitSignedTo_Negative11)
{
    uint8_t number = 0;
    number |= (1UL << (0));
    number |= (1UL << (2));
    number |= (1UL << (4));

    int8_t answer = get_bits<int8_t>(&number, 3, 5);
    ASSERT_EQ(answer, -11);
}

TEST(IncreaseSignedTypeSize, FiveBitSignedTo_5)
{
    uint8_t number = 0;
    number |= 1UL << 0;
    number |= 1UL << 2;

    int8_t answer = get_bits<int8_t>(&number, 3, 5);
    ASSERT_EQ(answer, 5);
}

TEST(IncreaseSignedTypeSize, SixBitsSignedTo_Negative11)
{
    uint8_t six_bits = 0;
    six_bits |= 1UL << 0;
    six_bits |= 1UL << 2;
    six_bits |= 1UL << 4;
    six_bits |= 1UL << 5;
    
    int8_t answer = get_bits<int8_t>(&six_bits, 2, 6);
    ASSERT_EQ(answer, -11);
}