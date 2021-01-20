#include <tuple>
#include <gtest/gtest.h>
#include "Configuration/configuration_keys.hpp"

using ublox::configuration::get_data_storage_size;
using ublox::configuration::get_group_id;
using ublox::configuration::get_item_id;

typedef std::tuple<uint32_t, uint8_t> data_tuple;

class GetDataStorageSize: public ::testing::TestWithParam<data_tuple>
{};

TEST_P(GetDataStorageSize, TestCorrectMessageClass)
{
    data_tuple key_data_size_pair = GetParam();
    ASSERT_EQ(get_data_storage_size(std::get<0>(key_data_size_pair)), std::get<1>(key_data_size_pair));
}

INSTANTIATE_TEST_SUITE_P(KeysAndDataSizes, GetDataStorageSize,
    ::testing::Values
    (
        data_tuple(0x10240012, 1),
        data_tuple(0x20240011, 2),
        data_tuple(0x20240014, 2),
        data_tuple(0x10240020, 1),
        data_tuple(0x40240021, 4),
        data_tuple(0x40240023, 4),
        data_tuple(0x20920005, 2),
        data_tuple(0x10410013, 1),
        data_tuple(0x30de0006, 3),
        data_tuple(0x30de0007, 3),
        data_tuple(0x10110013, 1)
    )
);


typedef std::tuple<uint32_t, uint8_t> group_tuple;

class GetGroupID: public ::testing::TestWithParam<group_tuple>
{};

TEST_P(GetGroupID, TestCorrectGroupID)
{
    group_tuple key_group_pair = GetParam();
    ASSERT_EQ(get_group_id(std::get<0>(key_group_pair)), std::get<1>(key_group_pair));
}

INSTANTIATE_TEST_SUITE_P(KeysAndGroupIDs, GetGroupID,
    ::testing::Values
    (
        // CFG-BDS
        data_tuple(0x10340014, 0x34),
        // CFG-GEOFENCE
        data_tuple(0x10240012, 0x24),
        data_tuple(0x20240011, 0x24),
        data_tuple(0x20240014, 0x24),
        data_tuple(0x10240020, 0x24),
        data_tuple(0x40240021, 0x24),
        data_tuple(0x40240023, 0x24),
        // CFG-HW
        data_tuple(0x10a3002e, 0xa3),
        // CFG-INFMSG
        data_tuple(0x20920005, 0x92),
        // CFG-ITFM
        data_tuple(0x10410013, 0x41),
        data_tuple(0x30de0006, 0xde),
        data_tuple(0x30de0007, 0xde),
        data_tuple(0x10110013, 0x11)
    )
);

typedef std::tuple<uint32_t, uint16_t> item_tuple;

class GetItemID: public ::testing::TestWithParam<item_tuple>
{};

TEST_P(GetItemID, TestCorrectItemID)
{
    item_tuple key_item_pair = GetParam();
    ASSERT_EQ(get_item_id(std::get<0>(key_item_pair)), std::get<1>(key_item_pair));
}

INSTANTIATE_TEST_SUITE_P(KeysAndItemIDs, GetItemID,
    ::testing::Values
    (
        // CFG-BDS
        data_tuple(0x10340014, 0x014),
        // CFG-GEOFENCE
        data_tuple(0x10240012, 0x012),
        data_tuple(0x20240011, 0x011),
        data_tuple(0x20240014, 0x014),
        data_tuple(0x10240020, 0x020),
        data_tuple(0x40240021, 0x021),
        data_tuple(0x40240023, 0x023),
        // CFG-HW
        data_tuple(0x10a3002e, 0x02e),
        // CFG-INFMSG
        data_tuple(0x20920005, 0x005),
        // CFG-ITFM
        data_tuple(0x10410013, 0x013),
        data_tuple(0x30de0006, 0x006),
        data_tuple(0x30de0007, 0x007),
        data_tuple(0x10110013, 0x013)
    )
);