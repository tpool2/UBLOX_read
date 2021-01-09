#include "GNSS/gnss.hpp"

namespace gnss
{

constellation_t Satellite::get_constellation() const
{
    return constellation;
}

int SatelliteDatabase::update_satellite(int constellation, int satellite)
{
    return 0;
}

namespace gps
{

}

}