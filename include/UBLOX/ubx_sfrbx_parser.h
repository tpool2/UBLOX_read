#ifndef UBX_SFRBX_PARSER
#define UBX_SFRBX_PARSER
#include "UBLOX/ubx_defs.h"
#include "UBLOX/ubx_listener.h"
namespace ublox::ubx
{
    class SFRBX_Parser: public MessageParser
    {
        private:
        public:
            SFRBX_Parser()
            {
                message_class = kCLASS_RXM;
                message_id = kRXM_SFRBX;
            }
            void read_byte(uint8_t byte);
            void reset();
            void finish_message();
    };
}

#endif