#include <gtest/gtest.h>
#include "GNSS/gps_lnav.hpp"
#include <iostream>
#include <tuple>
#include <vector>

using std::make_tuple;

namespace gnss { namespace gps { namespace lnav {

uint32_t sat_03[]
{
    0X22c05d10, 0X28ccaa9c,
    0Xc82c7d6,  0X8dba513c,
    0X3b050cf8, 0X286c084,
    0X38d4535b, 0X38ea845,
    0X837eca97, 0X1ec31f97
};

uint32_t sat_07[]
{
    0x22c05d10, 0x28ccaa9c,
    0x7ff6203,  0xb955c46,
    0xa1ac8191, 0xbf7f4076,
    0xaac9c2fe, 0x8457284c,
    0x3553e54,  0x1ec31fc4
};

uint32_t sat_22[]
{
    0x22c05d10, 0x28ccaa9c,
    0x5ffa71a,  0x8c413efc,
    0x1a5aa601, 0xbf9d00f0,
    0x89fd611,  0x81d2287b,
    0x38ff847,  0x1ec2d090
};

uint32_t sat_26[]
{
    0x22c05d10, 0x28ccaa9c,
    0x147f9c40, 0xbf57e8a,
    0x8a37d001, 0xbfa6c10a,
    0xa3b58606, 0x81c5686d,
    0x8359f2a6, 0x9ec31f48
};

uint32_t sat_27[]
{
    0x22c05d10, 0x28ccaa9c,
    0x7bf552b,  0xddb7420,
    0x26e0deac, 0x3f7440c8,
    0x21c78ff0, 0x461684a,
    0x833e1c26, 0x9ec31f1b
};

uint32_t bad_03[]
{
    0X22c05d10, 0X28ccaa9c,
    0Xc82c7d6,  0X8dba513c,
    0X3b050cf8, 0X286c084,
    0X38d4535b, 0X38ea845,
    0X837eca97, 0X1ec31f98
};

uint32_t bad_07[]
{
    0x22c05d10, 0x28ccaa9c,
    0x7ff6203,  0xb955c46,
    0xa1ac8191, 0xbf7f4076,
    0xaac9c2fe, 0x8457284e,
    0x3553e54,  0x1ec31fc4
};

uint32_t bad_22[]
{   
    0x22c05d10, 0x28ccaa9c, 
    0x5ffa71a,  0x8c413efc,
    0x1a5aa601, 0xbf9d00f0,
    0x89fd611,  0x81d2287b,
    0x38ff846,  0x1ec2d090  
};

uint32_t bad_26[]
{
    0x22c05d10, 0x28ccaa9c,
    0x147f9c40, 0xbf57e8a,
    0x8a37d001, 0xbfa6c10b,
    0xa3b58606, 0x81c5686d,
    0x8359f2a6, 0x9ec31f48
};

uint32_t bad_27[]
{
    0x22c05d10, 0x28ccaa9c,
    0x7bf552a,  0xddb7420,
    0x26e0deac, 0x3f7440c8,
    0x21c78ff0, 0x461684a,
    0x833e1c26, 0x9ec31f1b
};

std::vector<std::tuple<int, int>> bit_indicies
{
    make_tuple(0, 2),   make_tuple(1, 3),   make_tuple(27, 29),     make_tuple(28, 30),     make_tuple(29, 31),
    make_tuple(30, 34), make_tuple(31, 35), make_tuple(57, 61),     make_tuple(58, 62),     make_tuple(59, 63),
    make_tuple(60, 66), make_tuple(61, 67), make_tuple(87, 93),     make_tuple(88, 94),     make_tuple(89, 95),
    make_tuple(90, 98), make_tuple(91, 99), make_tuple(117, 125),   make_tuple(118, 126),   make_tuple(119, 127)   
};

class UBLOX_LNAV: public ::testing::TestWithParam<uint32_t*>
{
    protected:
        uint32_t* words;
        void SetUp() override
        {
            words = GetParam();
        }
};

TEST_P(UBLOX_LNAV, Preamble_Check)
{
    ASSERT_EQ(bit_utils::get_msb_bits<uint8_t>(words, 2, 8), gnss::gps::kPreamble);
}

TEST_P(UBLOX_LNAV, Full_Parity_Check_True)
{
    ASSERT_TRUE(check_parity(words));
}

INSTANTIATE_TEST_SUITE_P(NavData, UBLOX_LNAV,
testing::Values
(
    sat_03, sat_07, sat_22, sat_26, sat_27
));

class UBLOX_LNAV_BAD: public ::testing::TestWithParam<uint32_t*>
{
    protected:
        uint32_t* words;
        void SetUp() override
        {
            words = GetParam();
        }
};

TEST_P(UBLOX_LNAV_BAD, Full_Parity_Check_False)
{
    ASSERT_FALSE(check_parity(words));
}

INSTANTIATE_TEST_SUITE_P(BadNavData, UBLOX_LNAV_BAD,
testing::Values
(
    bad_03, bad_07, bad_22, bad_26, bad_27
));

class UBLOX_LNAV_With_Word_Index: public ::testing::TestWithParam<std::tuple<uint32_t*, int>>
{
    protected:
        uint32_t* words;
        int word_index;
        void SetUp() override
        {
            auto params = GetParam();
            words = std::get<0>(params);
            word_index = std::get<1>(params);
        }
};

TEST_P(UBLOX_LNAV_With_Word_Index, Parity_Check_True)
{
    bool D_29, D_30;
    if(word_index > 0)
    {
        int previous_word_index = word_index-1;
        D_29 = bit_utils::get_msb_bits<bool>(&words[previous_word_index], 30, 1);
        D_30 = bit_utils::get_msb_bits<bool>(&words[previous_word_index], 31, 1);
    }
    else
    {
        D_29 = 0;
        D_30 = 0;
    }
    ASSERT_TRUE(check_parity(words[word_index], D_29, D_30));
}

INSTANTIATE_TEST_SUITE_P(NavData, UBLOX_LNAV_With_Word_Index,
testing::Combine
(
    testing::Values
    (
        sat_03, sat_07, sat_22, sat_26, sat_27
    ),
    testing::Range(0, 10)
));

class BitIndicies: public ::testing::TestWithParam<std::tuple<int,int>>
{
    protected:
        int l1_bit_index;
        int real_bit_index;
        void SetUp() override
        {
            auto params = GetParam();
            l1_bit_index = std::get<0>(params);
            real_bit_index = std::get<1>(params);
        }
};

TEST_P(BitIndicies, MatchingBitTrue)
{
    ASSERT_EQ(get_ublox_bit_index(l1_bit_index), real_bit_index);
}

INSTANTIATE_TEST_SUITE_P(BitIndiciesParams, BitIndicies,
testing::ValuesIn
(
    bit_indicies
));

class UBLOX_LNAV_With_Bit_Index: public ::testing::TestWithParam<std::tuple<std::tuple<int, int>, uint32_t*>>
{
    protected:
        int l1_bit_index;
        int real_bit_index;
        uint32_t* words;
        void SetUp() override
        {
            auto params = GetParam();
            l1_bit_index = std::get<0>(std::get<0>(params));
            real_bit_index = std::get<1>(std::get<0>(params));
            words = std::get<1>(params);
        }
};

TEST_P(UBLOX_LNAV_With_Bit_Index, MatchingBitDataTrue)
{
    ASSERT_EQ(get_bits<bool>(words, l1_bit_index, 1), bit_utils::get_msb_bits<bool>(words, real_bit_index, 1));
}

INSTANTIATE_TEST_SUITE_P(BitIndiciesParams, UBLOX_LNAV_With_Bit_Index,
testing::Combine
(
    testing::ValuesIn(bit_indicies),
    testing::Values
    (
        sat_03, sat_07, sat_22, sat_26, sat_27
    )
));

} // namespace lnav
} // namespace gps
} // namespace gnss