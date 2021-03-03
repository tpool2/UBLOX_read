#include "bit_utils/bit_utils.h"
#include "GNSS/gps_lnav.hpp"

using bit_utils::get_msb_bits;

namespace gnss { namespace gps { namespace lnav {

bool check_parity(uint32_t* words)
{
    bool D_29 = 0;
    bool D_30 = 0;
    for(int word_index = 0; word_index < gps::kWordsPerSubframe; ++word_index)
    {
        if(!check_parity(words[word_index], D_29, D_30))
        {
            return false;
        }
        else
        {
            D_29 = bit_utils::get_msb_bits<bool>(&words[word_index], 30, 1);
            D_30 = bit_utils::get_msb_bits<bool>(&words[word_index], 31, 1);
        }
    }
    return true;
}

bool check_parity(uint32_t word, bool D_29_star, bool D_30_star)
{
    bool D[gps::kBitsPerWord];
    for(int i = 0; i < gps::kBitsPerWord; ++i)
    {
        D[i] = get_msb_bits<bool>(&word, i+2, 1);
    }
    
    bool d[24];
    for(int i = 0; i < 24; ++i)
    {
        d[i] = D[i];
    }

    if(D_30_star)
    {
        D_30_star = !D_30_star;
        D_29_star = !D_29_star;
    }

    bool D_25 = D_29_star^d[0]^d[1]^d[2]^d[4]^d[5]^d[9]^d[10]^d[11]^d[12]^d[13]^d[16]^d[17]^d[19]^d[22];
    bool D_26 = D_30_star^d[1]^d[2]^d[3]^d[5]^d[6]^d[10]^d[11]^d[12]^d[13]^d[14]^d[17]^d[18]^d[20]^d[23];
    bool D_27 = D_29_star^d[0]^d[2]^d[3]^d[4]^d[6]^d[7]^d[11]^d[12]^d[13]^d[14]^d[15]^d[18]^d[19]^d[21];
    bool D_28 = D_30_star^d[1]^d[3]^d[4]^d[5]^d[7]^d[8]^d[12]^d[13]^d[14]^d[15]^d[16]^d[19]^d[20]^d[22];
    bool D_29 = D_30_star^d[0]^d[2]^d[4]^d[5]^d[6]^d[8]^d[9]^d[13]^d[14]^d[15]^d[16]^d[17]^d[20]^d[21]^d[23];
    bool D_30 = D_29_star^d[2]^d[4]^d[5]^d[7]^d[8]^d[9]^d[10]^d[12]^d[14]^d[18]^d[21]^d[22]^d[23];
    return D_25 == D[24] && D_26 == D[25] && D_27 == D[26] && D_28 == D[27] && D_29 == D[28] && D_30 == D[29];
}

int get_ublox_bit_index(int l1_desired_bit_index)
{
    return l1_desired_bit_index+2*(int(l1_desired_bit_index/30)+1);
}

void LNAVParser::read_subframe(const uint32_t* subframe)
{
    subframe_id = get_bits<uint8_t>(&subframe[1], 19, 3);
    switch(subframe_id)
    {
        case 1:
            read_subframe_1(subframe);
            break;
        case 2:
            read_subframe_2(subframe);
            break;
        case 3:
            read_subframe_3(subframe);
            break;
    };
}

void LNAVParser::read_subframe_1(const uint32_t* subframe)
{
    WN = get_bits<uint16_t>(subframe, 60, 10);
    T_GD = double(get_bits<int64_t>(subframe, 196, 8))*powf128(2, -31);
    a_f0 = double(get_bits<int64_t>(subframe, 270, 22))*powf128(2, -31);
    a_f1 = double(get_bits<int64_t>(subframe, 248, 16))*powf128(2, -43);
    a_f2 = double(get_bits<int64_t>(subframe, 240, 8))*powf128(2, -55);
}

void LNAVParser::read_subframe_2(const uint32_t* subframe)
{
    C_rs = double(get_bits<int64_t>(subframe, 68, 16))*powf128(2, -5);
    Delta_n = double(get_bits<int16_t>(subframe, 90, 16))*powf128(2, -43);
    M_0 = double((get_bits<int64_t>(subframe, 106, 8)<<24) + get_bits<int64_t>(subframe, 120, 24))*powf128(2, -31);
    C_uc = double(get_bits<int16_t>(subframe, 150, 16))*powf128(2, -29);
    e = double((get_bits<int32_t>(subframe, 166, 8)<<24) | get_bits<int32_t>(subframe, 180, 24))*powf128(2, -33);
    C_us = double(get_bits<int16_t>(subframe, 210, 16))*powf128(2, -29);
    sqrt_A = double((get_bits<uint32_t>(subframe, 226, 8) << 24) + get_bits<uint32_t>(subframe, 240, 24))*powf128(2, -19);
    t_oe = get_bits<uint64_t>(subframe, 270, 16) << 4;
}

void LNAVParser::read_subframe_3(const uint32_t* subframe)
{
    IODE = get_bits<int>(subframe, 270, 8);
    C_ic = double(get_bits<int16_t>(subframe, 60, 16))*powf128(2, -29);
    Omega_0 = double( (get_bits<int32_t>(subframe, 76, 8)<<24) | get_bits<uint32_t>(subframe, 90, 24))*powf128(2, -43);
    C_is = double(get_bits<int16_t>(subframe, 120, 16))*powf128(2, -29);
    // i_0
    C_rc = double(get_bits<int16_t>(subframe, 180, 16))*powf128(2, -5);
    // omega
    Omega_dot = double(get_bits<int32_t>(subframe, 240, 24))*powf128(2, -43);
    // IDOT = double(get_bits<int16_t>())
}

} // namespace lnav
} // namespace gps
} // namespace gnss