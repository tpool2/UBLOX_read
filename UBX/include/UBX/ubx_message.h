#ifndef UBX_MESSAGE
#define UBX_MESSAGE
#include <cstring>
#include "UBX/ubx_defs.h"

namespace ublox { namespace ubx
{
    class UBX_message_t
    {
        public:
            UBX_message_t()
            {
                memset(&buffer, 0, UBX_MESSAGE_BUFFER_SIZE);
            }
            union
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
            };
            uint8_t get_checksum_a() const;
            uint8_t get_checksum_b() const;
            void update_checksums();
    };

    typedef std::function<void(const UBX_message_t&)> ubx_callback_function;
    
    UBX_message_t create_message(uint8_t msg_class, uint8_t id, uint16_t payload_length, const uint8_t* payload);
} }

#endif