#ifndef GPS_DEFS
#define GPS_DEFS
#include <cstdint>

namespace gnss { namespace gps
{
    enum
    {
        kWordLength = 30,
        kSubframeLength = 10,
        kPreamble = 139,
    };

    static void parse_l1_ca(const uint32_t* words);
    static void parse_subframe_3(const uint32_t* words);
    static void parse_l2c(const uint32_t* words);
    static void parse_l2_nav_11(const uint32_t* words);
    static int32_t l1_get_split_data(const uint32_t* words, int position);
} }

#endif