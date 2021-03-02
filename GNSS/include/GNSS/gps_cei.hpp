#ifndef GPS_CEI_HPP
#define GPS_CEI_HPP
#include <cstdint>
#include <iostream>
#include "GNSS/gps_lnav.hpp"

namespace gnss { namespace gps {

struct CEI
{
    uint8_t IODE;
    double C_rs;
    double delta_n;
    double M_0;
    double C_uc;
    double e;
    double C_us;
    double sqrt_A;
    double t_oe;
    double C_ic;
    double Omega_0;
    double C_is;
    double i_0;
    double C_rc;
    double omega;
    double Omega_dot;
    double IDOT;
};

namespace lnav
{
    bool update(CEI&, uint32_t* words);
}
} // namespace gps
} // namespace gnss
#endif