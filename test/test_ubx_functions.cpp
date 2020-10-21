#include <gtest/gtest.h>
#include "UBLOX/ubx_functions.h"
#include "UBLOX/ubx_defs.h"

TEST(TestMessageLengthVerification, 2_2_0xReturnTrue)
{
    ASSERT_TRUE(ublox::ubx::verify_message_length(2, 2, 0));
}

TEST(TestMessageLengthVerification, 2_1_2xReturnFalse)
{
    ASSERT_FALSE(ublox::ubx::verify_message_length(2, 1, 2));
}

TEST(TestMessageLengthVerification, 10_6_4xReturnTrue)
{
    ASSERT_TRUE(ublox::ubx::verify_message_length(10,6,4));
}