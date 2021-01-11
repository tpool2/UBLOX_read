#ifndef GPS_DEFS
#define GPS_DEFS
#include <cstdint>
#include <cmath>

namespace gnss { namespace gps
{
    enum
    {
        kWordLength = 30,
        kSubframeLength = 10,
        kPreamble = 139,
    };

    static double const mu = 3.986005*pow(10, 14);
    static double const Omega_e_dot = 7.2921151467*pow(10, -5);
    static double const pi = 3.1415926535898;

    static void parse_l1_ca(const uint32_t* words);
    static void parse_subframe_3(const uint32_t* words);
    static void parse_L2(const uint32_t* words);
    static void parse_L2_10(const uint32_t* words);
    static void parse_L2_11(const uint32_t* words);
    static int32_t l1_get_split_data(const uint32_t* words, int position);
} }

#endif