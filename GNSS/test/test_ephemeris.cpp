#include <gtest/gtest.h>
#include "GNSS/gnss.hpp"

#include <memory>

using gnss::EphemerisInterface;
using gnss::gps::CNAVEphemeris;

TEST(Ephemeris, ParseEphemerisDataToGetLocation)
{
    std::shared_ptr<EphemerisInterface> eph = std::make_shared<CNAVEphemeris>();
    eph->update_location();
}