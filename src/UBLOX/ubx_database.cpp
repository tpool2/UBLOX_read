#include "UBLOX/ubx_database.h"

bool ublox::ubx::DatabaseInterface::has(uint8_t message_class, uint8_t message_id)
{
    return false;
}

bool ublox::ubx::Database::has(uint8_t message_class, uint8_t message_id)
{
    return UBX_CLASS_Map.count(message_class) == 1
        && UBX_CLASS_Map[message_class].count(message_id) == 1;
}