#include <gtest/gtest.h>
#include "GNSS/satellite.hpp"

using gnss::SatelliteDatabase;
using gnss::gps::GPS_Satellite;
using gnss::galileo::Galileo_Satellite;

TEST(Satellite, Construct)
{
    std::shared_ptr<Satellite> gps_satellite = std::make_shared<GPS_Satellite>(2);
    ASSERT_EQ(gps_satellite->get_id(), 2);
}

TEST(Satellite, ConstructDefault)
{
    std::shared_ptr<Satellite> sat = std::make_shared<GPS_Satellite>();
    ASSERT_EQ(gps_satellite->get_id(), 0);
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

