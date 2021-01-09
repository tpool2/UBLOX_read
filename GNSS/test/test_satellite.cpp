#include <gtest/gtest.h>
#include "GNSS/gnss.hpp"

using gnss::Satellite;
using gnss::gps::GPS_Satellite;
using gnss::galileo::Galileo_Satellite;

TEST(Satellite, Construct)
{
    std::shared_ptr<Satellite> gps_satellite = std::make_shared<GPS_Satellite>();
}

TEST(Satellite, GPS_Constellation)
{
    std::shared_ptr<Satellite> satellite = std::make_shared<GPS_Satellite>();
    ASSERT_EQ(satellite->get_constellation(), gnss::kGPS);
}

TEST(Satellite, Galileo_Constellation)
{
    std::shared_ptr<Satellite> satellite = std::make_shared<Galileo_Satellite>();
    ASSERT_EQ(satellite->get_constellation(), gnss::kGALILEO);
}