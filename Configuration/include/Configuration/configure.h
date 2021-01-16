#ifndef CONFIGURE_H
#define CONFIGURE_H
#include <chrono>
#include <memory>
#include "async_comm/serial.h"
#include "UBX/ubx.hpp"

namespace ublox { namespace configure
{

double seconds_elapsed(const clock_t&);

ubx::UBX_message_t cfg_val_get_message(uint32_t configuration_key);
bool val_set(std::shared_ptr<async_comm::Serial>);
bool val_get(std::shared_ptr<async_comm::Serial>);

} // namespace configure
} // namespace ublox

#endif