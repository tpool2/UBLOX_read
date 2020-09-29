#include <gtest/gtest.h>
#include "UBLOX/ubx_defs.h"
#include "UBLOX/ubx_database.h"

using namespace ublox::ubx;

class TestDatabaseInterface: public ::testing::Test
{
    protected:
        ublox::ubx::DatabaseInterface interface;
};

TEST_F(TestDatabaseInterface, ReturnsFalse)
{
    ASSERT_FALSE(interface.has(0,1));
}

class TestDatabase: public ::testing::Test
{
    protected:
        ublox::ubx::Database database;
};

TEST_F(TestDatabase, HasACK_ACK)
{
    ASSERT_TRUE(database.has(ublox::ubx::kCLASS_ACK, ublox::ubx::kACK_ACK));
}

TEST_F(TestDatabase, DoesNotHaveACK_0x10)
{
    ASSERT_FALSE(database.has(ublox::ubx::kCLASS_ACK, 0x10));
}

TEST_F(TestDatabase, HasMGA_GPS)
{
    ASSERT_TRUE(database.has(kCLASS_MGA, kMGA_GPS));
}