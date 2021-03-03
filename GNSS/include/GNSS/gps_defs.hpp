#ifndef GPS_DEFS
#define GPS_DEFS
#include <cstdint>
#include <cmath>

namespace gnss { namespace gps
{
    enum
    {
        kBitsPerWord = 30,
        kWordsPerSubframe = 10,
        kSubframesPerMessage = 5,
        kPreamble = 139,
        kWordsPerMessage = 50,
    };

    static double const mu = 3.986005e14;
    static double const Omega_e_dot = 7.2921151467e-5;
    static double const pi = 3.1415926535898;
    static double const A_ref = 26559710; // meters
    static double const Omegad_ref = -2.6E-9; // semi-circles/second
} }

#endif