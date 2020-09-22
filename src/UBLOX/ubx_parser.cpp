#include "UBLOX/ubx_defs.h"
#include "UBLOX/ubx_parser.h"

int ublox::ubx::Parser::get_parser_state() const
{
    return parser_state;
}