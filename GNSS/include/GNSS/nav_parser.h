#ifndef NAV_PARSER
#define NAV_PARSER
#include <bitset>
#include "UBX/ubx.hpp"
#include "gps_defs.h"
#include "Bit_Utils/bit_utils.h"

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
        void parse_handover_word(const std::bitset<30> &word);
    private:
};
bool check_parity(const std::bitset<gps::kWordLength> &bits, const bool &prev_29, const bool &prev_30);
}
#endif