#include <gtest/gtest.h>
#include "GNSS/gps_lnav.hpp"
#include <iostream>
#include <tuple>

uint32_t sat_03[]{  0X22c05d10, 0X28ccaa9c,
                    0Xc82c7d6,  0X8dba513c,
                    0X3b050cf8, 0X286c084,
                    0X38d4535b, 0X38ea845,
                    0X837eca97, 0X1ec31f97  };

uint32_t sat_07[]{  0x22c05d10, 0x28ccaa9c,
                    0x7ff6203,  0xb955c46,
                    0xa1ac8191, 0xbf7f4076,
                    0xaac9c2fe, 0x8457284c,
                    0x3553e54,  0x1ec31fc4  };

uint32_t sat_22[]{  0x22c05d10, 0x28ccaa9c, 
                    0x5ffa71a,  0x8c413efc,
                    0x1a5aa601, 0xbf9d00f0,
                    0x89fd611,  0x81d2287b,
                    0x38ff847,  0x1ec2d090  };

uint32_t sat_26[]{  0x22c05d10, 0x28ccaa9c,
                    0x147f9c40, 0xbf57e8a,
                    0x8a37d001, 0xbfa6c10a,
                    0xa3b58606, 0x81c5686d,
                    0x8359f2a6, 0x9ec31f48  };

uint32_t sat_27[]{  0x22c05d10, 0x28ccaa9c,
                    0x7bf552b,  0xddb7420,
                    0x26e0deac, 0x3f7440c8,
                    0x21c78ff0, 0x461684a,
                    0x833e1c26, 0x9ec31f1b };

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

INSTANTIATE_TEST_SUITE_P(NavData, UBLOX_LNAV,
testing::Values
(
    sat_03, sat_07, sat_22, sat_26, sat_27
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

TEST_P(UBLOX_LNAV_With_Word_Index, Parity_Check)
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
    ASSERT_TRUE(gnss::gps::lnav::check_parity(words[word_index], D_29, D_30));
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