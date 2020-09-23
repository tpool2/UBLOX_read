#include <gtest/gtest.h>
#include "UBLOX/ubx_parser.h"

TEST(TestParserStates, StartByte1_0xb5)
{
    ASSERT_EQ(ublox::ubx::kStartByte_1, 0xb5);
}

TEST(TestParserStates, StartByte2_0x62)
{
    ASSERT_EQ(ublox::ubx::kStartByte_2, 0x62);
}

TEST(TestParserStates, SendStartByte1_GetParserState_GotStartByte1)
{
    ublox::ubx::Parser parser;
    parser.read_byte(ublox::ubx::kStartByte_1);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotStartByte_1);
}

TEST(TestParserStates, Reset_Send0x63_GetParserStateReset)
{
    ublox::ubx::Parser parser;
    parser.read_byte(0x63);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}