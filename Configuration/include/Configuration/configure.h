#ifndef CONFIGURE_H
#define CONFIGURE_H
#include <chrono>
#include <memory>
#include "async_comm/serial.h"
#include "UBX/ubx.hpp"
#include "ublox/ublox.hpp"

namespace ublox { namespace configure
{

double seconds_elapsed(const clock_t&);

ubx::UBX_message_t cfg_val_get_message(uint32_t configuration_key);
bool val_set(std::shared_ptr<Ublox>, uint32_t cfg_key, uint64_t cfg_data, uint8_t layer=ubx::CFG_VALSET_t::kRAM);
bool val_get(std::shared_ptr<async_comm::Serial>);

} // namespace configure
} // namespace ublox

#endif