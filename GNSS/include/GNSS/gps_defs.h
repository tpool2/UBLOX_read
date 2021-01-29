#ifndef GPS_DEFS
#define GPS_DEFS
#include <cstdint>
#include <cmath>
#include "ephemeris.hpp"

namespace gnss { namespace gps
{
    enum
    {
        kWordLength = 30,
        kSubframeLength = 10,
        kPreamble = 139,
    };

    static double const mu = 3.986005e14;
    static double const Omega_e_dot = 7.2921151467e-5;
    static double const pi = 3.1415926535898;
    static double const A_ref = 26559710; // meters
    static double const Omegad_ref = -2.6E-9; // semi-circles/second

    static void parse_l1_ca(const uint32_t* words);
    static void parse_subframe_3(const uint32_t* words);
    static message_10 parse_L2_10(const uint32_t* words);
    static message_11 parse_L2_11(const uint32_t* words);
    static int32_t l1_get_split_data(const uint32_t* words, int position);
} }

#endif