#include "UBLOX/ubx_sfrbx_parser.h"
#include <iostream>

namespace ublox::ubx
{
    void SFRBX_Parser::read_byte(uint8_t byte)
    {
        std::cout<<int(byte)<<std::endl;
    }

    void SFRBX_Parser::reset()
    {

    }

    void SFRBX_Parser::finish_message()
    {

    }
}