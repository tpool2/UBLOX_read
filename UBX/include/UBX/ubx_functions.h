#ifndef UBX_FUNCTIONS_H
#define UBX_FUNCTIONS_H
#include <memory>
#include <cstdint>

namespace ublox { namespace ubx
{
    bool verify_message_length(uint16_t length, int constant_coefficient, int linear_growth_coefficient);
}}

#endif