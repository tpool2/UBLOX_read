#include <gtest/gtest.h>
#include "UBLOX/ubx_parser.h"

class ParserResetStateTestFixture: public ::testing::Test
{
    protected:
        ublox::ubx::Parser parser;
};

TEST(TestParserStates, StartByte1_0xb5)
{
    ASSERT_EQ(ublox::ubx::kStartByte_1, 0xb5);
}

TEST(TestParserStates, StartByte2_0x62)
{
    ASSERT_EQ(ublox::ubx::kStartByte_2, 0x62);
}

TEST_F(ParserResetStateTestFixture, SendStartByte1_GetParserState_GotStartByte1)
{
    parser.read_byte(ublox::ubx::kStartByte_1);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotStartByte_1);
}

TEST_F(ParserResetStateTestFixture, Reset_Send0x63_GetParserStateReset)
{
    parser.read_byte(0x63);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}

class ParserGotStartByte1: public ::testing::Test
{
    protected:
        ublox::ubx::Parser parser;
        void SetUp() override
        {
            parser.read_byte(ublox::ubx::kStartByte_1);
        }
};

TEST_F(ParserGotStartByte1, SendStartByte2_GetParserState_GotStartByte2)
{
    parser.read_byte(ublox::ubx::kStartByte_2);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotStartByte_2);
}

TEST_F(ParserGotStartByte1, Send0x63_GetParserState_Reset)
{
    parser.read_byte(0x63);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}

class ParserGotStartByte2: public ::testing::Test
{
    protected:
        ublox::ubx::Parser parser;
        void SetUp() override
        {
            parser.read_byte(ublox::ubx::kStartByte_1);
            parser.read_byte(ublox::ubx::kStartByte_2);
        }
};

TEST_F(ParserGotStartByte2, SendACKMessageClass_GetStateGotMessageClass)
{
    parser.read_byte(ublox::ubx::kCLASS_ACK);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageClass);
}

TEST_F(ParserGotStartByte2, SendCFGMessageClass_GetStateGotMessageClass)
{
    parser.read_byte(ublox::ubx::kCLASS_CFG);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageClass);
}

TEST_F(ParserGotStartByte2, SendINFMessageClass_GetStateGotMessageClass)
{
    parser.read_byte(ublox::ubx::kCLASS_INF);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageClass);
}

TEST_F(ParserGotStartByte2, SendLOGMessageClass_GetStateGotMessageCLASS_)
{
    parser.read_byte(ublox::ubx::kCLASS_LOG);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageClass);
}

TEST_F(ParserGotStartByte2, SendMGAMessageCLASS__GetStateGotMessageClass)
{
    parser.read_byte(ublox::ubx::kCLASS_MGA);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageClass);
}

TEST_F(ParserGotStartByte2, SendMONMessageClass_GetStateGotMessageClass)
{
    parser.read_byte(ublox::ubx::kCLASS_MON);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageClass);
}

TEST_F(ParserGotStartByte2, SendNAVMessageClass_GetStateGotMessageClass)
{
    parser.read_byte(ublox::ubx::kCLASS_NAV);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageClass);
}

class ParserGotMessageClass: public ::testing::Test
{
    protected:
        ublox::ubx::Parser parser;
        void SetUp() override
        {
            parser.read_byte(ublox::ubx::kStartByte_1);
            parser.read_byte(ublox::ubx::kStartByte_2);
            parser.read_byte(ublox::ubx::kCLASS_ACK);
        }
};

TEST_F(ParserGotMessageClass, SendMessageTypeACK_ACK)
{
    parser.read_byte(ublox::ubx::kACK_ACK);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageID);
}

TEST_F(ParserGotMessageClass, SendMessageTypeACK_NACK)
{
    parser.read_byte(ublox::ubx::kACK_NACK);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageID);
}

TEST_F(ParserGotMessageClass, SendMessageTypeNAV_ORB)
{
    parser.read_byte(ublox::ubx::kNAV_ORB);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}