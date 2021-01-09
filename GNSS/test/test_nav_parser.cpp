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