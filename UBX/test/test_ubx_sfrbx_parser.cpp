#include <gtest/gtest.h>
#include <memory>
#include "UBX/ubx.hpp"

using namespace ublox::ubx;

class TestParser: public ::testing::Test
{
    protected:
        std::shared_ptr<MessageParser> message_parser;
        void SetUp() override
        {
            message_parser = std::make_shared<SFRBX_Parser>();
        }
};

TEST_F(TestParser, GetMessageClass)
{
    ASSERT_EQ(message_parser->get_message_class(), kCLASS_RXM);
}

TEST_F(TestParser, GetMessageID)
{
    ASSERT_EQ(message_parser->get_message_id(), kRXM_SFRBX);
}