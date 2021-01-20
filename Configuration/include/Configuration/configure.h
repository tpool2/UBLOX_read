#ifndef CONFIGURE_H
#define CONFIGURE_H
#include <chrono>
#include <memory>
#include "async_comm/serial.h"
#include "UBX/ubx.hpp"
#include "ublox/ublox.hpp"
#include "bit_utils/bit_utils.h"

namespace ublox { namespace configure
{

double seconds_elapsed(const clock_t&);

template <class T> ubx::UBX_message_t cfg_val_set_message(uint32_t configuration_key, T configuration_data, uint8_t layer = ubx::CFG_VALSET_t::kRAM)
{
    using bit_utils::get_lsb_bits;
    
    uint16_t payload_length = 8 + sizeof(T);
    uint8_t payload[payload_length];
    memset(payload, 0, payload_length);
    payload[0] = ubx::CFG_VALSET_t::kVERSION_0;
    payload[1] = layer;
    payload[2] = 0;
    payload[3] = 0;
    payload[4] = get_lsb_bits<uint8_t>(&configuration_key, 0, 8);
    payload[5] = get_lsb_bits<uint8_t>(&configuration_key, 8, 8);
    payload[6] = get_lsb_bits<uint8_t>(&configuration_key, 16, 8);
    payload[7] = get_lsb_bits<uint8_t>(&configuration_key, 24, 8);
    for(int index = 8; index < payload_length; ++index)
    {
        payload[index] = get_lsb_bits<uint8_t>(&configuration_data, 8*(index-8), 8);
    }
    return ubx::create_message(ubx::kCLASS_CFG, ubx::kCFG_VALSET, payload_length, payload);
}
ubx::UBX_message_t cfg_val_get_message(uint32_t configuration_key);
bool val_set(std::shared_ptr<Ublox>, uint32_t cfg_key, uint8_t cfg_data, uint8_t layer=ubx::CFG_VALSET_t::kRAM);
bool val_get(std::shared_ptr<async_comm::Serial>);

} // namespace configure
} // namespace ublox

#endif