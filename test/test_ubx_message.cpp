#include <gtest/gtest.h>
#include "UBLOX/ubx_message.h"

using namespace ublox::ubx;

TEST(UBX_Message, Create)
{
    UBX_Message message;
}

TEST(UBX_Message, GetBytesStartByte1)
{
    UBX_Message message;
    uint8_t* bytes = message.get_bytes();
    ASSERT_EQ(*bytes, kStartByte_1);
}