#include "UBLOX/ubx_defs.h"
#include "UBLOX/ubx_parser.h"

using std::uint8_t;

namespace ublox::ubx
{
    void Parser::get_start_byte_1()
    {
        if(current_byte == kStartByte_1)
            ++parser_state;
    }

    void Parser::get_start_byte_2()
    {
        advance_or_reset(current_byte == kStartByte_2);
    }

    void Parser::get_length_2()
    {
        message_length += (current_byte << kByteSize);
        advance_or_reset(database->get_node(message_class, message_id)->length_matches(message_length));
    }
    
    void Parser::check_message_class_id()
    {
        advance_or_reset(database->has(message_class, message_id));
    }

    void Parser::advance_or_reset(bool advance)
    {
        if(advance)
        {
            this->advance();
        }
        else
        {
            reset();
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

    void Parser::get_payload()
    {
        if(payload_bytes_received < message_length-1)
        {
            ++payload_bytes_received;
            update_checksum();
        }
        else if(payload_bytes_received == message_length-1)
        {
            ++payload_bytes_received;
            advance();
        }
        else
        {
            reset();
        }
    }

    void Parser::verify_checksum_a()
    {
        advance_or_reset(current_byte==checksum_a);
    }

    void Parser::verify_checksum_b()
    {
        advance_or_reset(current_byte==checksum_b);
    }

    void Parser::finish_message()
    {
        reset();
    }

    void Parser::read_byte(const uint8_t& byte)
    {
        current_byte = byte;
        switch(parser_state)
        {
            case kReset:
                get_start_byte_1();
                break;
            
            case kGotStartByte_1:
                get_start_byte_2();
                break;

            case kGotStartByte_2:
                message_class = current_byte;
                advance();
                break;

            case kGotMessageClass:
                message_id = current_byte;
                check_message_class_id();
                break;

            case kGotMessageID:
                advance();
                message_length = current_byte;
                break;

            case kGotLength_1:
                get_length_2();
                break;

            case kGotLength_2:
                get_payload();
                break;

            case kGotPayload:
                verify_checksum_a();
                break;

            case kGotChecksumA:
                verify_checksum_b();
                if(parser_state==kGotChecksumB)
                {
                    finish_message();
                }
                break;

            default:
                reset();
                break;
        }
    }
}