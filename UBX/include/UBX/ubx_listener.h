#ifndef UBX_LISTENER
#define UBX_LISTENER
#include <cstdint>

namespace ublox{ namespace ubx
{
    class MessageParser
    {
        protected:
            uint8_t message_class = 0;
            uint8_t message_id = 0;
        
        public:
            virtual void read_byte(uint8_t byte) = 0;
            virtual void reset() = 0;
            virtual void finish_message() = 0;
            uint8_t get_message_class() const;
            uint8_t get_message_id() const;

    };
}}

#endif