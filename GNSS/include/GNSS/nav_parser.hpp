#ifndef NAV_PARSER
#define NAV_PARSER
#include <iostream>
#include "UBX/ubx.hpp"
#include "gps_defs.hpp"

namespace gnss
{
class NavParser
{
    public:
        ublox::ubx::ubx_callback_function callback = [this](const ublox::ubx::UBX_message_t& message)
        {
            this->parse_sfrbx(message);
        };
        void parse_sfrbx(const ublox::ubx::UBX_message_t&);
        void read_gps_message(const uint32_t* buffer, size_t len);
    private:
};
}
#endif