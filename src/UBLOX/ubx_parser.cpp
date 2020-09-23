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
        }
    }

}