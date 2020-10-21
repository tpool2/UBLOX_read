#include "UBLOX/ubx_functions.h"
namespace ublox::ubx
{
    bool verify_message_length(uint16_t length, int constant_coefficient, int linear_growth_coefficient)
    {
        int remainder = int(length) - constant_coefficient;
        if(linear_growth_coefficient != 0)
        {
            return remainder%linear_growth_coefficient == 0;
        }
        else
        {
            return remainder == 0;
        }
    }
}