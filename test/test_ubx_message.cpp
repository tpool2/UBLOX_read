#include <gtest/gtest.h>
#include "UBLOX/ubx_message.h"

using namespace ublox::ubx;

TEST(CreateUBXMessage, ACK_ACK)
{
    uint8_t payload[2] = {kCLASS_CFG,kCFG_VALDEL};
    UBX_message_t message = create_message(kCLASS_ACK, kACK_ACK, 2, payload);
    ASSERT_EQ(message.message_class, kCLASS_ACK);
    ASSERT_EQ(message.message_id, kACK_ACK);
    ASSERT_EQ(message.payload_length, 2);
    ASSERT_EQ(message.payload.buffer[0], kCLASS_CFG);
    ASSERT_EQ(message.payload.buffer[1], kCFG_VALDEL);
}

TEST(CreateUBXMessage, ACK_ACK_Checksum_A)
{
    uint8_t payload[2] = {kCLASS_CFG,kCFG_VALDEL};
    UBX_message_t message = create_message(kCLASS_ACK, kACK_ACK, 2, payload);
    ASSERT_EQ(message.get_checksum_a(), 154);
}

TEST(CreateUBXMessage, ACK_ACK_Checksum_B)
{
    uint8_t payload[2] = {kCLASS_CFG,kCFG_VALDEL};
    UBX_message_t message = create_message(kCLASS_ACK, kACK_ACK, 2, payload);
    ASSERT_EQ(message.get_checksum_b(), 195);
}

class CreateCFGVALGETMessage: public ::testing::Test
{
    protected:
        UBX_message_t cfg_message;
        uint16_t my_position = 0x0010;
        uint32_t my_key_id = CFG_VALGET_t::kMSGOUT_SFRBX;

        void SetUp()
        {
            union
            {
                CFG_VALGET_t::request_t request;
                uint8_t payload[8];
            };

            request.version = CFG_VALGET_t::kREQUEST;
            request.layer = CFG_VALGET_t::kRAM;
            request.position = my_position;
            request.cfgDataKey.keyID = my_key_id;

            cfg_message = create_message(kCLASS_CFG, kCFG_VALGET, 8, payload);
        }
};

TEST_F(CreateCFGVALGETMessage, TestkCLASSCFG)
{
    ASSERT_EQ(cfg_message.message_class, kCLASS_CFG);
}

TEST_F(CreateCFGVALGETMessage, TestkCFG_VALGET)
{
    ASSERT_EQ(cfg_message.message_id, kCFG_VALGET);
}

TEST_F(CreateCFGVALGETMessage, TestLength)
{
    ASSERT_EQ(cfg_message.payload_length, 8);
}

TEST_F(CreateCFGVALGETMessage, TestVersion)
{
    ASSERT_EQ(cfg_message.payload.CFG_VALGET.version, CFG_VALGET_t::kREQUEST);
}

TEST_F(CreateCFGVALGETMessage, TestLayer)
{
    ASSERT_EQ(cfg_message.payload.CFG_VALGET.layer, CFG_VALGET_t::kRAM);
}

TEST_F(CreateCFGVALGETMessage, TestPosition)
{
    ASSERT_EQ(cfg_message.payload.CFG_VALGET.position, my_position);
}

TEST_F(CreateCFGVALGETMessage, TestKeyID)
{
    ASSERT_EQ(cfg_message.payload.CFG_VALGET.cfgDataKey.keyID, my_key_id);
}