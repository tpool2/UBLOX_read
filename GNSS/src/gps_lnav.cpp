#include "bit_utils/bit_utils.h"

#include "GNSS/gps_lnav.hpp"

using bit_utils::get_msb_bits;

namespace gnss { namespace gps { namespace lnav {

bool check_parity(uint32_t* words)
{
    bool D_29 = 0;
    bool D_30 = 0;
    for(int word_index = 0; word_index < gps::kSubframeLength; ++word_index)
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
    bool D[gps::kWordLength];
    for(int i = 0; i < gps::kWordLength; ++i)
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

} // namespace lnav
} // namespace gps
} // namespace gnss