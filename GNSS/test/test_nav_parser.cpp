#include <vector>
#include <gtest/gtest.h>
#include "GNSS/gnss.hpp"

using namespace gnss;
TEST(NavParser, Construct)
{
    NavParser parser;
}

TEST(Parse_SFRBX, gnssID)
{
    NavParser parser;
    ublox::ubx::UBX_message_t message;
    message.payload.RXM_SFRBX.gnssId = ublox::ubx::kGnssID_GPS;
    message.payload.RXM_SFRBX.svId = 1;
    message.payload.RXM_SFRBX.numWords = 10;
    parser.parse_sfrbx(message);
}

class CheckParityGPS: public ::testing::TestWithParam<std::string>
{
    protected:

};

TEST_P(CheckParityGPS, Set1)
{
    ASSERT_TRUE(gnss::check_parity(std::bitset<30>(GetParam()), 0, 0));
}

INSTANTIATE_TEST_SUITE_P(GPS, CheckParityGPS,
    testing::Values
    (
        "100010110000000101000000101110",
        "011100010100001100110101101000",
        "010101110000100011011001001000",
        "011110110000110001110100100100",
        "111111010101000000000000110111",
        "101000010000110010011010100110",
        "001110110101011101010110110010",
        "010100101101011010011110010100",
        "000111011110010010100001001110",
        "000010110000000001011110110111"
    )
);