#include "GNSS/gps_cei.hpp"

namespace gnss { namespace gps { 

std::string CEI::to_string() const
{
    std::stringstream ss;
    ss << "SV Health: " << SV_Health << std::endl;
    ss << "IODC: " << IODC << std::endl;
    ss << "URA: " << URA << std::endl;
    ss << "WN: " << WN << std::endl;
    ss << "T_GD: " << T_GD << std::endl;
    ss << "a_f0: " << a_f0 << std::endl;
    ss << "a_f1: " << a_f1 << std::endl;
    ss << "a_f2: " << a_f2 << std::endl;
    ss << "t_oc: " << t_oc << std::endl;
    ss << "sqrt_A: " << sqrt_A << std::endl;
    ss << "Delta_n: " << Delta_n << std::endl;
    ss << "e: " << e << std::endl;
    ss << "M_0: " << M_0 << std::endl;
    ss << "t_oe: " << t_oe << std::endl;
    ss << "C_rs: " << C_rs << std::endl;
    ss << "C_uc: " << C_uc << std::endl;
    ss << "C_us: " << C_us << std::endl;
    ss << "IODE: " << IODE << std::endl;
    ss << "omega: " << omega << std::endl;
    ss << "Omega_dot: " << Omega_dot << std::endl;
    ss << "Omega_0: " << Omega_0 << std::endl;
    ss << "i_0: " << i_0 << std::endl;
    ss << "IDOT: " << IDOT << std::endl;
    ss << "C_ic: " << C_ic << std::endl;
    ss << "C_is: " << C_is << std::endl;
    ss << "C_rc: " << C_rc << std::endl;

    return ss.str();
}

} // namespace gps
} // namespace gnss