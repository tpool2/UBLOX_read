#ifndef GPS_LNAV_HPP
#define GPS_LNAV_HPP
#include "GNSS/gps_defs.h"
#include "bit_utils/bit_utils.h"
namespace gnss { namespace gps { namespace lnav {



/*
bool check_parity
Checks the parity of a given GPS word given the previous word's 29th and 30th bits
Assumes that the first two most significant bits are not part of the data but are rather
for padding
Inputs:
uint32_t word: GPS word
bool D_29_star: 29th bit of previous word in subframe
bool D_30_star: 30th bit of previous word in subframe
*/
bool check_parity(uint32_t word, bool D_29, bool D_30);

bool check_parity(uint32_t* words);

int get_ublox_bit_index(int l1_desired_bit_index);

template <class T, class R> T get_bits(const R* buffer, int position, int length = sizeof(T))
{
    return bit_utils::get_msb_bits<T>(buffer, get_ublox_bit_index(position), length);
}

} // namespace lnav
} // namespace gps
} // namespace gnss

#endif