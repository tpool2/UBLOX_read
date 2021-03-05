#ifndef GPS_LNAV_HPP
#define GPS_LNAV_HPP
#include "bit_utils/bit_utils.h"
#include "GNSS/gps_defs.hpp"
#include "GNSS/gps_cei.hpp"
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

template <class T, class R> T get_bits(const R* lnav_words, int position, int length = sizeof(T))
{
    return bit_utils::get_msb_bits<T>(lnav_words, get_ublox_bit_index(position), length);
}

template <class T, class R> T get_spliced_bits(const R* lnav_words, int start_1, int len_1, int start_2, int len_2)
{
    int ublox_start_1 = get_ublox_bit_index(start_1);
    int ublox_start_2 = get_ublox_bit_index(start_2);
    return bit_utils::splice_msb_bits<T>(lnav_words, ublox_start_1, len_1, ublox_start_2, len_2);
}

class LNAVParser: public CEI
{
    private:
        
    public:
        uint8_t subframe_id = 0;
        void read_subframe(const uint32_t* subframe);
        void read_subframe_1(const uint32_t* subframe);
        void read_subframe_2(const uint32_t* subframe);
        void read_subframe_3(const uint32_t* subframe);
};

} // namespace lnav
} // namespace gps
} // namespace gnss

#endif