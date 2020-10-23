#include "UBLOX/ubx_database.h"

namespace ublox::ubx
{
    bool Database::DatabaseNode::length_matches(int payload_length) const
    {
        int remainder = payload_length - length;
        if(linear_length != 0 && remainder >=0)
        {
            return remainder%linear_length == 0;
        }
        else
        {
            return remainder == 0;
        }
    }

    void Database::DatabaseNode::finish_messages() const
    {
        for(auto message_parser : message_parsers)
        {
            message_parser->finish_message();
        }
    }

    void Database::DatabaseNode::pass_byte(uint8_t byte) const
    {
        for(auto message_parser : message_parsers)
        {
            message_parser->read_byte(byte);
        }
    }

    void Database::DatabaseNode::add_message_parser(std::shared_ptr<MessageParser> message_parser)
    {
        message_parsers.push_back(message_parser);
    }

    bool Database::has(uint8_t message_class, uint8_t message_id)
    {
        return UBX_CLASS_Map.count(message_class) == 1
            && UBX_CLASS_Map[message_class].count(message_id) == 1;
    }

    std::shared_ptr<DatabaseNodeInterface> Database::get_node(uint8_t message_class, uint8_t message_id)
    {
        std::shared_ptr<DatabaseNodeInterface> return_node;
        if(has(message_class, message_id))
        {
            return_node = UBX_CLASS_Map[message_class][message_id];
        }
        else
        {
            return_node = null_node;
        }
        
        return return_node;
    }
}