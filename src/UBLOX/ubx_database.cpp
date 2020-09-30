#include "UBLOX/ubx_database.h"

bool ublox::ubx::DatabaseInterface::has(uint8_t message_class, uint8_t message_id)
{
    return false;
}

uint16_t ublox::ubx::DatabaseInterface::get_length(uint8_t message_class, uint8_t message_id)
{
    return 0;
}

uint16_t ublox::ubx::Database::DatabaseNode::get_length() const
{
    return length;
}

bool ublox::ubx::Database::has(uint8_t message_class, uint8_t message_id)
{
    return UBX_CLASS_Map.count(message_class) == 1
        && UBX_CLASS_Map[message_class].count(message_id) == 1;
}

uint16_t ublox::ubx::Database::get_length(uint8_t message_class, uint8_t message_id)
{
    return UBX_CLASS_Map[message_class][message_id].get_length();
}