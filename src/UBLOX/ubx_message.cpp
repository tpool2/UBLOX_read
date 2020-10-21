#include "UBLOX/ubx_message.h"

using namespace ublox::ubx;

uint8_t* UBX_Message::get_bytes() const
{
    uint8_t bytes[8];
    bytes[0] = kStartByte_1;
    
    return bytes;
}