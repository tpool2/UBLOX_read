#include "GNSS/nav_parser.hpp"

namespace gnss
{
    void NavParser::parse_sfrbx(const ublox::ubx::UBX_message_t& message)
    {
        uint8_t gnss_id = message.payload.RXM_SFRBX.gnssId;
        if(gnss_id == ublox::ubx::kGnssID_GPS)
        {
            // read_gps_message(message.payload.RXM_SFRBX.dwrd, message.payload.RXM_SFRBX.numWords);
        }
        else
        {
            
        }
    }
}