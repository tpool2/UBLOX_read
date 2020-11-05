#include "UBLOX/ubx_message.h"

namespace ublox::ubx
{
    UBX_message_t create_message(uint8_t msg_class, uint8_t id, uint16_t payload_length, const uint16_t* payload)
    {
        UBX_message_t message;
        message.start_byte_1 = kSTART_BYTE_1;
        message.start_byte_2 = kSTART_BYTE_2;
        message.message_class = msg_class;
        message.message_id = id;
        message.payload_length = payload_length;
        for(int index = 0; index < payload_length; ++index)
        {
            message.payload.buffer[index] = payload[index];
        }
        message.update_checksums();
        return message;
    }

    void UBX_message_t::update_checksums()
    {
        for(int start = 2; start < 6+payload_length; ++start)
        {
            payload.buffer[payload_length] += buffer[start];
            payload.buffer[payload_length+1] += payload.buffer[payload_length];
        }
    }

    uint8_t UBX_message_t::get_checksum_a() const
    {
        return payload.buffer[payload_length];
    }

    uint8_t UBX_message_t::get_checksum_b() const
    {
        return payload.buffer[payload_length+1];
    }
}