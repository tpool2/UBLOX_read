#ifndef NAV_PARSER
#define NAV_PARSER
#include <bitset>
#include "UBLOX/ubx_defs.h"
#include "GNSS/gps_defs.h"
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
        void read_gps_message(const uint8_t* buffer, size_t len);
    private:
};

bool check_parity(const std::bitset<gps::kWordLength> &bits, const bool &prev_29, const bool &prev_30);
}
#endif