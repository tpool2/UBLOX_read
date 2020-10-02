#include <gtest/gtest.h>
#include "UBLOX/ubx_defs.h"
#include "UBLOX/ubx_database.h"

using namespace ublox::ubx;

class TestDatabase: public ::testing::Test
{
    
    protected:
        ublox::ubx::Database database;
        std::shared_ptr<DatabaseNodeInterface> orb_node;
        void SetUp() override
        {
            orb_node = database.get_node(kCLASS_NAV, kNAV_ORB);
        }
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

TEST_F(TestDatabase, MatchLengthACK_ACK)
{
    auto node = database.get_node(kCLASS_ACK,kACK_ACK);
    ASSERT_TRUE(node->length_matches(2));
    ASSERT_FALSE(node->length_matches(4));
}

TEST_F(TestDatabase, MatchLengthNAV_ORB)
{
    ASSERT_TRUE(orb_node->length_matches(8));
    ASSERT_TRUE(orb_node->length_matches(14));
}

TEST_F(TestDatabase, FindNonExistantNode)
{
    auto node = database.get_node(0x11,0x0a);
    ASSERT_TRUE(node->length_matches(0));
}

TEST_F(TestDatabase, PreventNegativeLength)
{
    ASSERT_FALSE(orb_node->length_matches(2));
}