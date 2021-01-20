#include <memory>
#include <gtest/gtest.h>
#include "UBX/ubx.hpp"

using namespace ublox::ubx;

class ParserResetStateTestFixture: public ::testing::Test
{
    protected:
        ublox::ubx::Parser parser;
};

TEST(TestParserStates, StartByte1_0xb5)
{
    ASSERT_EQ(ublox::ubx::kSTART_BYTE_1, 0xb5);
}

TEST(TestParserStates, StartByte2_0x62)
{
    ASSERT_EQ(ublox::ubx::kSTART_BYTE_2, 0x62);
}

TEST_F(ParserResetStateTestFixture, SendStartByte1_GetParserState_GotStartByte1)
{
    ASSERT_TRUE(parser.read_byte(ublox::ubx::kSTART_BYTE_1));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotStartByte_1);
}

TEST_F(ParserResetStateTestFixture, Reset_Send0x63_GetParserStateReset)
{
    ASSERT_FALSE(parser.read_byte(0x63));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}

class ParserGotStartByte1: public ::testing::Test
{
    protected:
        ublox::ubx::Parser parser;
        void SetUp() override
        {
            parser.read_byte(ublox::ubx::kSTART_BYTE_1);
        }
};

TEST_F(ParserGotStartByte1, SendStartByte2_GetParserState_GotStartByte2)
{
    ASSERT_TRUE(parser.read_byte(ublox::ubx::kSTART_BYTE_2));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotStartByte_2);
}

TEST_F(ParserGotStartByte1, Send0x63_GetParserState_Reset)
{
    ASSERT_FALSE(parser.read_byte(0x63));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}

class ParserGotStartByte2: public ::testing::TestWithParam<uint8_t>
{
    protected:
        ublox::ubx::Parser parser;
        void SetUp() override
        {
            parser.read_byte(ublox::ubx::kSTART_BYTE_1);
            parser.read_byte(ublox::ubx::kSTART_BYTE_2);
        }
};

class ParserCorrectMessageClass: public ParserGotStartByte2 {};
TEST_P(ParserCorrectMessageClass, SendMessageClasses)
{
    ASSERT_TRUE(parser.read_byte(GetParam()));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageClass);
}
INSTANTIATE_TEST_SUITE_P(MessageClasses, ParserCorrectMessageClass,
    testing::Values
        (
        kCLASS_ACK, kCLASS_INF, kCLASS_INF, kCLASS_LOG, kCLASS_MGA,
        kCLASS_MON, kCLASS_NAV, kCLASS_RXM, kCLASS_SEC, kCLASS_TIM, kCLASS_UPD
        )
);

class ParserIncorrectMessageClass: public ParserGotStartByte2 {};
TEST_P(ParserIncorrectMessageClass, SendMessageClasses)
{
    ASSERT_FALSE(parser.read_byte(GetParam()));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}
INSTANTIATE_TEST_SUITE_P(MessageClasses, ParserIncorrectMessageClass,
    testing::Values
        (
        0x22, 0x03, 0x0e, 0x10
        )
);

class ParserGotMessageClass: public ::testing::Test
{
    protected:
        ublox::ubx::Parser parser;
        void SetUp() override
        {
            parser.read_byte(ublox::ubx::kSTART_BYTE_1);
            parser.read_byte(ublox::ubx::kSTART_BYTE_2);
            parser.read_byte(ublox::ubx::kCLASS_ACK);
        }
};

TEST_F(ParserGotMessageClass, SendMessageTypeACK_ACK)
{
    ASSERT_TRUE(parser.read_byte(ublox::ubx::kACK_ACK));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageID);
}

TEST_F(ParserGotMessageClass, SendMessageTypeACK_NACK)
{
    ASSERT_TRUE(parser.read_byte(ublox::ubx::kACK_NACK));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotMessageID);
}

TEST_F(ParserGotMessageClass, SendMessageTypeNAV_ORB)
{
    ASSERT_FALSE(parser.read_byte(ublox::ubx::kNAV_ORB));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}

class ParserGotMessageClassIDACK_ACK: public ::testing::Test
{
    protected:
        ublox::ubx::Parser parser;
        void SetUp() override
        {
            parser.read_byte(ublox::ubx::kSTART_BYTE_1);
            parser.read_byte(ublox::ubx::kSTART_BYTE_2);
            parser.read_byte(ublox::ubx::kCLASS_ACK);
            parser.read_byte(ublox::ubx::kACK_ACK);
        }
};

TEST_F(ParserGotMessageClassIDACK_ACK, SendCorrectMessageLength_ParserStateGotLength_2)
{
    ASSERT_TRUE(parser.read_byte(0x02));
    ASSERT_TRUE(parser.read_byte(0x00));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotLength_2);
}

TEST_F(ParserGotMessageClassIDACK_ACK, SendIncorrectMessageLength_ParserStateReset)
{
    ASSERT_TRUE(parser.read_byte(0x02));
    ASSERT_TRUE(parser.read_byte(0x02));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}

class ParserGotMessageClassIDNAV_ORB: public ::testing::Test
{
    protected:
        ublox::ubx::Parser parser;
        void SetUp() override
        {
            parser.read_byte(ublox::ubx::kSTART_BYTE_1);
            parser.read_byte(ublox::ubx::kSTART_BYTE_2);
            parser.read_byte(ublox::ubx::kCLASS_NAV);
            parser.read_byte(ublox::ubx::kNAV_ORB);
        }
};

TEST_F(ParserGotMessageClassIDNAV_ORB, SendCorrectMessageLength_ParserStateGotLength_2)
{
    parser.read_byte(14);
    parser.read_byte(0);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotLength_2);
}

TEST_F(ParserGotMessageClassIDNAV_ORB, SendWrongMessageLength_ParserStateReset)
{
    parser.read_byte(2);
    parser.read_byte(0);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}

class ParserCallbacks: public ::testing::Test
{
    protected:
        Parser parser;
        bool parsed_ack_ack = false;
        bool parsed_ack_nack = false;
        void SetUp() override
        {
            parser.register_callback(kCLASS_ACK, kACK_NACK, [this](const UBX_message_t& ubx_message)
            {
                this->ack_nack_cb(ubx_message);
            });
            parser.register_callback(kCLASS_ACK, kACK_ACK, [this](const UBX_message_t& ubx_message)
            {
                this->ack_ack_cb(ubx_message);
            });
            parser.read_byte(kSTART_BYTE_1);
            parser.read_byte(kSTART_BYTE_2);
            parser.read_byte(kCLASS_ACK);
            parser.read_byte(kACK_ACK);
            parser.read_byte(0x02);
            parser.read_byte(0x00);
            parser.read_byte(kCLASS_CFG);
            parser.read_byte(kCFG_VALDEL);
        }

        void parse_ack_ack()
        {
            parser.read_byte(kSTART_BYTE_1);
            parser.read_byte(kSTART_BYTE_2);
            parser.read_byte(kCLASS_ACK);
            parser.read_byte(kACK_ACK);
            parser.read_byte(0x02);
            parser.read_byte(0x00);
            parser.read_byte(kCLASS_CFG);
            parser.read_byte(kCFG_VALDEL);
            parser.read_byte(154);
            parser.read_byte(195);
        }

        void ack_ack_cb(const UBX_message_t& ubx_message)
        {
            parsed_ack_ack = true;
        }

        void ack_nack_cb(const UBX_message_t& ubx_message)
        {
            parsed_ack_nack = true;
        }
};

TEST_F(ParserCallbacks, ACK_ACK_Checksum_A)
{
    ASSERT_TRUE(parser.read_byte(154));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotChecksumA);
}

TEST_F(ParserCallbacks, ACK_ACK_Checksum_A_Wrong)
{
    ASSERT_FALSE(parser.read_byte(100));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}

TEST_F(ParserCallbacks, ACK_ACK_Checksum_B)
{
    parser.read_byte(154);
    ASSERT_TRUE(parser.read_byte(195));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}

TEST_F(ParserCallbacks, ACK_ACK_Checksum_B_Wrong)
{
    parser.read_byte(154);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotChecksumA);
    ASSERT_FALSE(parser.read_byte(100));
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}

TEST_F(ParserCallbacks, ACK_ACK_CB)
{
    parser.read_byte(154);
    parser.read_byte(195);
    ASSERT_TRUE(parsed_ack_ack);
    ASSERT_FALSE(parsed_ack_nack);

    parser.read_byte(kSTART_BYTE_1);
    parser.read_byte(kSTART_BYTE_2);
    parser.read_byte(kCLASS_ACK);
    parser.read_byte(kACK_NACK);
    parser.read_byte(0x02);
    parser.read_byte(0x00);
    parser.read_byte(kCLASS_CFG);
    parser.read_byte(kCFG_VALDEL);
    ASSERT_TRUE(parser.read_byte(153));
    ASSERT_TRUE(parser.read_byte(190));
    ASSERT_EQ(parser.get_parser_state(), Parser::kReset);
    ASSERT_TRUE(parsed_ack_nack);
}

TEST_F(ParserCallbacks, ACK_ACK_INFORMATION)
{
    parser.register_callback(kCLASS_ACK, kACK_ACK,[](const UBX_message_t message)
    {
        ASSERT_EQ(message.start_byte_1, kSTART_BYTE_1);
        ASSERT_EQ(message.start_byte_2, kSTART_BYTE_2);
        ASSERT_EQ(message.message_class, kCLASS_ACK);
        ASSERT_EQ(message.message_id, kACK_ACK);
        ASSERT_EQ(message.payload_length, 2);
        ASSERT_EQ(message.payload.ACK_ACK.clsID, kCLASS_CFG);
        ASSERT_EQ(message.payload.ACK_ACK.msgID, kCFG_VALDEL);
    });
    parser.read_byte(154);
    parser.read_byte(195);
}

TEST_F(ParserCallbacks, Deregister_ACK_ACK)
{
    parser.pop_callbacks(1);
    parser.read_byte(154);
    parser.read_byte(195);
    ASSERT_FALSE(parsed_ack_ack);
    parser.register_callback(ublox::ubx::kCLASS_ACK, ublox::ubx::kACK_ACK, [this](const ublox::ubx::UBX_message_t &msg)
    {
        parsed_ack_ack = true;
    });
    parse_ack_ack();
    ASSERT_TRUE(parsed_ack_ack);
}

class ParserGotMessageClassIDRXM_SFRBX: public ::testing::Test
{
    protected:
        ublox::ubx::Parser parser;
        void SetUp() override
        {
            parser.read_byte(ublox::ubx::kSTART_BYTE_1);
            parser.read_byte(ublox::ubx::kSTART_BYTE_2);
            parser.read_byte(ublox::ubx::kCLASS_RXM);
            parser.read_byte(ublox::ubx::kRXM_SFRBX);
        }

};

TEST_F(ParserGotMessageClassIDRXM_SFRBX, SendMessageLengthCorrect_ParserStateGotMessageLength)
{
    parser.read_byte(12);
    parser.read_byte(0);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotLength_2);
}

TEST_F(ParserGotMessageClassIDRXM_SFRBX, SendMessageLengthWrong_ParserStateReset)
{
    parser.read_byte(11);
    parser.read_byte(0);
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kReset);
}

TEST_F(ParserGotMessageClassIDRXM_SFRBX, SendMessageLengthAndPayload_ParserStateGotPayload)
{
    parser.read_byte(8);
    parser.read_byte(0);
    for(int i = 0; i < 8; ++i)
    {
        parser.read_byte(1);
    }
    ASSERT_EQ(parser.get_parser_state(), ublox::ubx::Parser::kGotPayload);
}