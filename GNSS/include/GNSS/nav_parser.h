#ifndef NAV_PARSER
#define NAV_PARSER
#include <bitset>
#include <map>
#include <memory>
#include "UBX/ubx.hpp"
#include "gps_defs.h"
#include "bit_utils/bit_utils.h"
#include "satellite.hpp"

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
        SatelliteDatabase database;
    private:
        std::map<uint8_t, std::shared_ptr<gps::message_10> > message_10_map;
        void parse_L2(const uint32_t* words);
};
bool check_parity(const std::bitset<gps::kWordLength> &bits, const bool &prev_29, const bool &prev_30);
}
#endif