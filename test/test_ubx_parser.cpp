#include <gtest/gtest.h>
#include "UBLOX/ubx_defs.h"

TEST(TestParserStates, StartByte1_0xb5)
{
    ASSERT_EQ(ublox::ubx::start_byte_1, 0xb5);
}

TEST(TestParserStates, StartByte2_0x62)
{
    ASSERT_EQ(ublox::ubx::start_byte_2, 0x62);
}

TEST(TestParserStates, SendStartByte1_GetParserState_GotStartByte1)
{
    ublox::ubx::Parser parser;
}