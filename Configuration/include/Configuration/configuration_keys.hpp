#ifndef CONFIGURATION_KEYS_HPP
#define CONFIGURATION_KEYS_HPP
#include <cstdint>
#include "bit_utils/bit_utils.h"

namespace ublox
{
namespace configuration
{

typedef uint32_t configuration_key_t;

uint8_t get_data_storage_size(const configuration_key_t& configuration_key);
uint8_t get_group_id(const configuration_key_t& configuration_key);
uint16_t get_item_id(const configuration_key_t& configuration_key);

} // namespace configuration
} // namespace ublox
#endif