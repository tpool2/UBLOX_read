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

class TestGNSS_Word: public ::testing::Test
{
    protected:
        std::bitset<gnss::gps::kWordLength> bits;
        void SetUp() override
        {
            memset(&bits, 0, sizeof(bits));
        }
};

TEST_F(TestGNSS_Word, Check_Parity)
{
    ASSERT_TRUE(gnss::check_parity(bits, 0, 0));
}