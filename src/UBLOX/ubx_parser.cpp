#include "UBLOX/ubx_defs.h"
#include "UBLOX/ubx_parser.h"
#include <iostream>

using std::uint8_t;

namespace ublox::ubx
{
    bool Parser::get_start_byte_1()
    {
        if(current_byte == kStartByte_1)
        {
            ++parser_state;
            return true;
        }
        else
        {
            reset();
            return false;
        }
    }

    bool Parser::get_start_byte_2()
    {
        if(current_byte == kStartByte_2)
        {
            ++parser_state;
            return true;
        }
        else
        {
            reset();
            return false;
        }
    }

    bool Parser::get_length_2()
    {
        message_length += (current_byte << kByteSize);
        return advance_or_reset(database->get_node(message_class, message_id)->length_matches(message_length));
    }
    
    bool Parser::check_message_class_id()
    {
        return advance_or_reset(database->has(message_class, message_id));
    }

    bool Parser::advance_or_reset(bool advance)
    {
        if(advance)
        {
            this->advance();
            return true;
        }
        else
        {
            reset();
            return false;
        }
    }

    void Parser::advance()
    {
        ++parser_state;
        update_checksum();
    }

    void Parser::update_checksum()
    {
        checksum_a += current_byte;
        checksum_b += checksum_a;
    }

    void Parser::reset()
    {
        parser_state = kReset;
        message_class = 0x00;
        message_id = 0x00;
        message_length = 0x0000;
        checksum_a = 0;
        payload_bytes_received = 0;
    }

    int Parser::get_parser_state() const
    {
        return parser_state;
    }

    bool Parser::get_payload()
    {
        if(payload_bytes_received < message_length-1)
        {
            ++payload_bytes_received;
            update_checksum();
            return true;
        }
        else if(payload_bytes_received == message_length-1)
        {
            ++payload_bytes_received;
            advance();
            return true;
        }
        else
        {
            reset();
            return false;
        }
    }

    bool Parser::verify_checksum_a()
    {
        return advance_or_reset(current_byte==checksum_a);
    }

    bool Parser::verify_checksum_b()
    {
        return advance_or_reset(current_byte==checksum_b);
    }

    void Parser::finish_message()
    {
        reset();
    }

    uint8_t Parser::get_checksum_a() const
    {
        return checksum_a;
    }

    bool Parser::read_byte(const uint8_t& byte)
    {
        current_byte = byte;
        bool byte_state = true;
        switch(parser_state)
        {
            case kReset:
                byte_state = get_start_byte_1();
                break;
            
            case kGotStartByte_1:
                byte_state = get_start_byte_2();
                break;

            case kGotStartByte_2:
                message_class = current_byte;
                advance();
                break;

            case kGotMessageClass:
                message_id = current_byte;
                byte_state = check_message_class_id();
                break;

            case kGotMessageID:
                advance();
                message_length = current_byte;
                break;

            case kGotLength_1:
                byte_state = get_length_2();
                break;

            case kGotLength_2:
                byte_state = get_payload();
                break;

            case kGotPayload:
                byte_state = verify_checksum_a();
                break;

            case kGotChecksumA:
                byte_state = verify_checksum_b();
                if(parser_state==kGotChecksumB)
                {
                    finish_message();
                }
                break;

            default:
                reset();
                byte_state = false;
                break;
        }
        return byte_state;
    }
}