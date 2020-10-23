#include "UBLOX/ubx_listener.h"

namespace ublox::ubx
{
    uint8_t MessageParser::get_message_class() const {return message_class;}
    uint8_t MessageParser::get_message_id() const {return message_id;}
}