#include "Configuration/configuration_keys.hpp"

namespace ublox
{
namespace configuration
{

uint8_t get_data_storage_size(const configuration_key_t& configuration_key)
{
    return bit_utils::get_lsb_bits<uint8_t>(&configuration_key, 28, 3);
}

} // namespace configuration
} // namespace ublox