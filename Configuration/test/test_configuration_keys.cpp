#include <tuple>
#include <gtest/gtest.h>
#include "Configuration/configuration_keys.hpp"

using ublox::configuration::get_data_storage_size;

typedef std::tuple<uint32_t, uint8_t> data_tuple;

class GetDataStorageSize: public ::testing::TestWithParam<data_tuple>
{

};

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
