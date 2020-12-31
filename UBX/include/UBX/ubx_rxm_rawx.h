#ifndef UBX_RXM_RAWX
#define UBX_RXM_RAWX
#include "UBX/ubx_listener.h"

namespace ublox { namespace ubx
{
    class RXM_RAWX_Parser: public MessageParser
    {
        private:

        
        public:
            void read_byte(uint8_t byte) override;
            void reset() override;
            void finish_message() override;
    };
}}

#endif