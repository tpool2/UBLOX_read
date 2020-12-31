#ifndef UBX_SFRBX_PARSER
#define UBX_SFRBX_PARSER
#include "UBX/ubx_defs.h"
#include "UBX/ubx_listener.h"
namespace ublox{ namespace ubx
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
}}

#endif