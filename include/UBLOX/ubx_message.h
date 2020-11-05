#ifndef UBX_MESSAGE
#define UBX_MESSAGE

#include "UBLOX/ubx_defs.h"

namespace ublox::ubx
{
    typedef union
    {
        uint8_t buffer[UBX_MESSAGE_BUFFER_SIZE];
        struct
        {
            uint8_t start_byte_1    : 8;
            uint8_t start_byte_2    : 8;
            uint8_t message_class   : 8;
            uint8_t message_id      : 8;
            uint16_t payload_length : 16;
            UBX_payload_t payload;
        };
    } UBX_message_t;
    
    typedef std::function<void(const UBX_message_t&)> ubx_callback_function;
    
    UBX_message_t create_message(uint8_t msg_class, uint8_t id, uint16_t payload_length, const uint16_t* payload);
}
#endif