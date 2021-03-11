#include <gtest/gtest.h>
#include "GNSS/gps_cnav.hpp"
namespace gnss { namespace gps { namespace cnav {

uint32_t sat_03[]
{
    0x8b0cba33, 0x23457d04,
    0x8bf909dd, 0x30d30060,
    0x21265fee, 0xfff5fffd,
    0xf601c8a,  0x6021d1ff,
    0xdca3b6ec, 0x1528b0de
};

uint32_t sat_04[]
{
    0x8b10aa32, 0x510ba719,
    0xf7457fc8, 0x3fffec5,
    0x3427000,  0x31ddc8d5,
    0xdab80461, 0x211a0f6f,
    0x4b920e2e, 0xda68b10b
};

uint32_t sat_07[]
{
    0x8b1cba33, 0x234527dd,
    0xc4209b1,  0xcdcd808e,
    0xef73dfe8, 0xbffd6002,
    0x6f201837, 0xe0273900,
    0x1f3a8107, 0x61e8b1de
};

uint32_t sat_08[]
{
    0x8b20ba33, 0x2344d1fe,
    0x4b8e09dd, 0xfbddfcb,
    0x9f6fe004, 0xc0127fff,
    0x5fc027d2, 0x800e20ff,
    0xf59f1973, 0x7a28b21e
};

uint32_t sat_09[]
{
    0x8b24ba33, 0x234511ff,
    0x733449b5, 0x1b581fec,
    0x4013dfff, 0x5fffdffe,
    0x58801e60, 0x201ce8ff,
    0xe74b69ac, 0xb068b25e
};

uint32_t sat_26[]
{
    0x8b68ba33, 0x23453c2e,
    0xfadd899c, 0xe5f89fb0,
    0x9fcf2008, 0x2004200b,
    0x68201d98, 0x401c8600,
    0xa0a9d1d4, 0xeda8b69e
};

class UBLOX_CNAV: public ::testing::TestWithParam<uint32_t*>
{
    protected:
        uint32_t* words;
        void SetUp() override
        {
            words = GetParam();
        }
};

TEST_P(UBLOX_CNAV, Preamble_Check)
{
    ASSERT_EQ(bit_utils::get_msb_bits<uint8_t>(words, 0, 8), gnss::gps::kPreamble);
}

INSTANTIATE_TEST_SUITE_P(NavData, UBLOX_CNAV,
testing::Values
(
    sat_03, sat_04, sat_07, sat_08, sat_09, sat_26
));
} // namespace cnav
} // namespace gps
} // namespace gnss