#ifndef UBX_MESSAGE
#define UBX_MESSAGE
#include "UBLOX/ubx_defs.h"

namespace ublox::ubx
{
    class UBX_Message
    {
        public:
            virtual uint8_t* get_bytes() const;
    };
}

#endif