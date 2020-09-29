#include "UBLOX/ubx_defs.h"
#include "UBLOX/ubx_parser.h"

using std::uint8_t;

namespace ublox::ubx
{

    int Parser::get_parser_state() const
    {
        return parser_state;
    }

    void Parser::read_byte(const uint8_t& byte)
    {
        switch(parser_state)
        {
            case kReset:
                if(byte == kStartByte_1)
                    ++parser_state;
                break;
            
            case kGotStartByte_1:
                if(byte == kStartByte_2)
                    ++parser_state;
                else
                {
                    parser_state = kReset;
                }
                break;

            case kGotStartByte_2:
                message_class = byte;
                ++parser_state;
                break;

            case kGotMessageClass:
                message_id = byte;
                if(database->has(message_class, message_id))
                {
                    ++parser_state;
                }
                else
                {
                    parser_state = kReset;
                    message_class = 0x00;
                    message_id = 0x00;
                }
                
                break;

            case kGotMessageID:
                ++parser_state;
                break;
        }
    }

}