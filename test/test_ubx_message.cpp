#include <gtest/gtest.h>
#include "UBLOX/ubx_message.h"

using namespace ublox::ubx;

TEST(CreateUBXMessage, ACK_ACK)
{
    uint16_t payload[2] = {kCLASS_CFG,kCFG_VALDEL};
    UBX_message_t message = create_message(kCLASS_ACK, kACK_ACK, 2, payload);
    ASSERT_EQ(message.message_class, kCLASS_ACK);
    ASSERT_EQ(message.message_id, kACK_ACK);
    ASSERT_EQ(message.payload_length, 2);
    ASSERT_EQ(message.payload.buffer[0], kCLASS_CFG);
    ASSERT_EQ(message.payload.buffer[1], kCFG_VALDEL);
}

TEST(CreateUBXMessage, ACK_ACK_Checksum_A)
{
    uint16_t payload[2] = {kCLASS_CFG,kCFG_VALDEL};
    UBX_message_t message = create_message(kCLASS_ACK, kACK_ACK, 2, payload);
    ASSERT_EQ(message.get_checksum_a(), 154);
}

TEST(CreateUBXMessage, ACK_ACK_Checksum_B)
{
    uint16_t payload[2] = {kCLASS_CFG,kCFG_VALDEL};
    UBX_message_t message = create_message(kCLASS_ACK, kACK_ACK, 2, payload);
    ASSERT_EQ(message.get_checksum_b(), 195);
}