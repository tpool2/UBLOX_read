#include <gtest/gtest.h>
#include "GNSS/gps_lnav.hpp"
#include <iostream>

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
    
};

TEST_P(UBLOX_LNAV, Preamble_Check)
{
    ASSERT_EQ(bit_utils::get_msb_bits<uint8_t>(GetParam(), 2, 8), gnss::gps::kPreamble);
}

TEST_P(UBLOX_LNAV, TLM_Parity_Check)
{
    ASSERT_TRUE(gnss::gps::lnav::check_parity(GetParam()[0], 0, 0));
}

TEST_P(UBLOX_LNAV, HOW_Parity_Check)
{
    uint32_t* words = GetParam();
    bool D_29 = bit_utils::get_msb_bits<bool>(&words[0], 30, 1);
    bool D_30 = bit_utils::get_msb_bits<bool>(&words[0], 31, 1);
    ASSERT_TRUE(gnss::gps::lnav::check_parity(GetParam()[1], D_29, D_30));
}

TEST_P(UBLOX_LNAV, Third_Word_Parity_Check)
{
    uint32_t* words = GetParam();
    bool D_29 = bit_utils::get_msb_bits<bool>(&words[1], 30, 1);
    bool D_30 = bit_utils::get_msb_bits<bool>(&words[1], 31, 1);
    ASSERT_TRUE(gnss::gps::lnav::check_parity(GetParam()[2], D_29, D_30));
}

TEST_P(UBLOX_LNAV, Fourth_Word_Parity_Check)
{
    uint32_t* words = GetParam();
    bool D_29 = bit_utils::get_msb_bits<bool>(&words[2], 30, 1);
    bool D_30 = bit_utils::get_msb_bits<bool>(&words[2], 31, 1);
    ASSERT_TRUE(gnss::gps::lnav::check_parity(GetParam()[3], D_29, D_30));
}

TEST_P(UBLOX_LNAV, Fifth_Word_Parity_Check)
{
    uint32_t* words = GetParam();
    bool D_29 = bit_utils::get_msb_bits<bool>(&words[3], 30, 1);
    bool D_30 = bit_utils::get_msb_bits<bool>(&words[3], 31, 1);
    ASSERT_TRUE(gnss::gps::lnav::check_parity(GetParam()[4], D_29, D_30));
}

TEST_P(UBLOX_LNAV, Sixth_Word_Parity_Check)
{
    uint32_t* words = GetParam();
    bool D_29 = bit_utils::get_msb_bits<bool>(&words[4], 30, 1);
    bool D_30 = bit_utils::get_msb_bits<bool>(&words[4], 31, 1);
    ASSERT_TRUE(gnss::gps::lnav::check_parity(GetParam()[5], D_29, D_30));
}

TEST_P(UBLOX_LNAV, Seventh_Word_Parity_Check)
{
    uint32_t* words = GetParam();
    bool D_29 = bit_utils::get_msb_bits<bool>(&words[5], 30, 1);
    bool D_30 = bit_utils::get_msb_bits<bool>(&words[5], 31, 1);
    ASSERT_TRUE(gnss::gps::lnav::check_parity(GetParam()[6], D_29, D_30));
}

TEST_P(UBLOX_LNAV, Eighth_Word_Parity_Check)
{
    uint32_t* words = GetParam();
    bool D_29 = bit_utils::get_msb_bits<bool>(&words[6], 30, 1);
    bool D_30 = bit_utils::get_msb_bits<bool>(&words[6], 31, 1);
    ASSERT_TRUE(gnss::gps::lnav::check_parity(GetParam()[7], D_29, D_30));
}

TEST_P(UBLOX_LNAV, Ninth_Word_Parity_Check)
{
    uint32_t* words = GetParam();
    bool D_29 = bit_utils::get_msb_bits<bool>(&words[7], 30, 1);
    bool D_30 = bit_utils::get_msb_bits<bool>(&words[7], 31, 1);
    ASSERT_TRUE(gnss::gps::lnav::check_parity(GetParam()[8], D_29, D_30));
}

TEST_P(UBLOX_LNAV, Tenth_Word_Parity_Check)
{
    uint32_t* words = GetParam();
    bool D_29 = bit_utils::get_msb_bits<bool>(&words[8], 30, 1);
    bool D_30 = bit_utils::get_msb_bits<bool>(&words[8], 31, 1);
    ASSERT_TRUE(gnss::gps::lnav::check_parity(GetParam()[9], D_29, D_30));
}

INSTANTIATE_TEST_SUITE_P(Sat_22, UBLOX_LNAV,
testing::Values
(
    sat_03, sat_07, sat_22, sat_26, sat_27
));