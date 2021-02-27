#include "GNSS/ephemeris.hpp"

namespace gnss{ 

double EphemerisInterface::get_x_ecef() const
{
    return x_ecef;
}

double EphemerisInterface::get_y_ecef() const
{
    return y_ecef;
}

double EphemerisInterface::get_z_ecef() const
{
    return z_ecef;
}

}