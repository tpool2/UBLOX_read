#include "UBLOX/ubx_database.h"

namespace ublox::ubx
{
    bool Database::DatabaseNode::length_matches(int payload_length) const
    {
        int remainder = payload_length - length;
        if(linear_length != 0)
        {
            return remainder%linear_length == 0;
        }
        else
        {
            return remainder == 0;
        }
    }

    bool Database::has(uint8_t message_class, uint8_t message_id)
    {
        return UBX_CLASS_Map.count(message_class) == 1
            && UBX_CLASS_Map[message_class].count(message_id) == 1;
    }

    std::shared_ptr<DatabaseNodeInterface> Database::get_node(uint8_t message_class, uint8_t message_id)
    {
        return UBX_CLASS_Map[message_class][message_id];
    }
}