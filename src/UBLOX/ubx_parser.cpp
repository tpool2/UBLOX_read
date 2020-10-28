#include "UBLOX/ubx_defs.h"
#include "UBLOX/ubx_parser.h"

using std::uint8_t;

namespace ublox::ubx
{
    void Parser::get_start_byte_1()
    {
        valid = advance_or_reset(current_byte == kSTART_BYTE_1);
    }

    void Parser::get_start_byte_2()
    {
        valid = advance_or_reset(current_byte == kSTART_BYTE_2);
    }

    void Parser::get_length_2()
    {
        payload_length += (current_byte << kByteSize);
        advance_or_reset(database->get_node(message_class, message_id)->length_matches(payload_length));
    }

    void Parser::check_message_class()
    {
        valid = advance_or_reset(database->has(message_class));
    }
    
    void Parser::check_message_class_id()
    {
        valid = advance_or_reset(database->has(message_class, message_id));
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
    }

    void Parser::reset()
    {
        parser_state = kReset;
        checksum_a = 0;
        checksum_b = 0;
        message_length = 0;
        memset(buffer, 0, BUFFER_SIZE);
    }

    int Parser::get_parser_state() const
    {
        return parser_state;
    }

    void Parser::receive_payload()
    {
        if(message_length == payload_length + 6)
        {
            advance();
            calculate_checksums();
        }
        valid = true;
    }

    void Parser::calculate_checksums()
    {
        for(int start = 2; start < 6+payload_length; ++start)
        {
            checksum_a += buffer[start];
            checksum_b += checksum_a;
        }
    }

    void Parser::register_callback(uint8_t message_class, uint8_t message_id, std::function<void(const uint8_t* payload, size_t size)> callback_function)
    {
        callbacks.push_back(Callback(message_class, message_id, callback_function));
    }

    void Parser::finish_message()
    {
        if(parser_state == kGotChecksumB)
        {
            for(auto callback : callbacks)
            {
                if(callback.matches(message_class, message_id))
                {
                    callback.callback_function(payload.buffer, message_length);
                }
            }
        }
        reset();
    }

    bool Parser::read_byte(const uint8_t& byte)
    {
        current_byte = byte;
        buffer[message_length] = current_byte;
        ++message_length;
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
                check_message_class();
                break;

            case kGotMessageClass:
                message_id = current_byte;
                check_message_class_id();
                break;

            case kGotMessageID:
                advance();
                payload_length = current_byte;
                break;

            case kGotLength_1:
                get_length_2();
                break;

            case kGotLength_2:
                receive_payload();
                break;

            case kGotPayload:
                valid = advance_or_reset(checksum_a==current_byte);
                break;
            
            case kGotChecksumA:
                valid = advance_or_reset(checksum_b==current_byte);
                finish_message();
                break;
        }
        return valid;
    }
}