#ifndef GPS_CEI_HPP
#define GPS_CEI_HPP
#include <cstdint>
#include <iostream>
#include <string>
#include <sstream>

namespace gnss { namespace gps {

class CEI
{
    public:
        uint8_t IODE = 0;
        double C_rs = 0;
        double Delta_n = 0;
        double M_0 = 0;
        double C_uc = 0;
        double e = 0;
        double C_us = 0;
        double sqrt_A = 0;
        double t_oe = 0;
        double C_ic = 0;
        double Omega_0 = 0;
        double C_is = 0;
        double i_0 = 0;
        double C_rc = 0;
        double omega = 0;
        double Omega_dot = 0;
        double IDOT = 0;
        double a_f0 = 0;
        double a_f1 = 0;
        double a_f2 = 0;
        int WN = 0;
        int IODC = 0;
        int URA = 0;
        int SV_Health = 0;
        double T_GD = 0;
        double t_oc = 0;
        std::string to_string() const;
};

} // namespace gps
} // namespace gnss
#endif