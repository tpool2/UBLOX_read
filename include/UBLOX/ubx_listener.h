#ifndef UBX_LISTENER
#define UBX_LISTENER
#include <cstdint>

namespace ublox::ubx
{
    class MessageParser
    {
        public:
            virtual void read_byte(uint8_t byte) = 0;
            virtual void reset() = 0;
            virtual void finish_message() = 0;
    };
}

#endif