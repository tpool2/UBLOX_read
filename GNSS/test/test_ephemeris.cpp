#include <gtest/gtest.h>
#include "GNSS/gnss.hpp"

#include <memory>

using gnss::EphemerisInterface;
using gnss::gps::L2Ephemeris;

TEST(Ephemeris, ParseEphemerisDataToGetLocation)
{
    std::shared_ptr<EphemerisInterface> eph = std::make_shared<L2Ephemeris>();
    eph->update_location();
}