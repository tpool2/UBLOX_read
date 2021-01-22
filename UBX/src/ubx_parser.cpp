#include "UBX/ubx.hpp"

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
        ubx_message.payload_length |= (static_cast<uint16_t>(current_byte) << kByteSize);
        advance_or_reset(database->get_node(ubx_message.message_class, ubx_message.message_id)->length_matches(ubx_message.payload_length));
    }

    void Parser::check_message_class()
    {
        valid = advance_or_reset(database->has(ubx_message.message_class));
    }
    
    void Parser::check_message_class_id()
    {
        valid = advance_or_reset(database->has(ubx_message.message_class, ubx_message.message_id));
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
        message_length = 0;
        memset(ubx_message.buffer, 0, BUFFER_SIZE+8);
    }

    int Parser::get_parser_state() const
    {
        return parser_state;
    }

    void Parser::receive_payload()
    {
        if(message_length == ubx_message.payload_length + 6)
        {
            advance();
            ubx_message.update_checksums();
        }
        valid = true;
    }

    void Parser::register_callback(uint8_t message_class, uint8_t message_id, std::function<void(const UBX_message_t&)> callback_function)
    {
        callbacks.push_back(Callback(message_class, message_id, callback_function));
    }

    void Parser::pop_callbacks(int number_of_callbacks)
    {
        for(int count = 0; count < number_of_callbacks; ++count)
        {
            callbacks.pop_back();
        }
    }

    void Parser::finish_message()
    {
        if(parser_state == kGotChecksumB)
        {
            std::cout<<"Message Class: "<<static_cast<uint16_t>(ubx_message.message_class)<<" Message ID: "<<static_cast<uint16_t>(ubx_message.message_id)<<std::endl;
            for(auto callback : callbacks)
            {
                if(callback.matches(ubx_message.message_class, ubx_message.message_id))
                {
                    callback.callback_function(ubx_message);
                }
            }
        }
        reset();
    }

    bool Parser::read_bytes(const uint8_t* buffer, size_t length)
    {
        bool status = true;
        for(int index = 0; index < length; ++index)
        {
            status &= read_byte(buffer[index]);
        }
        return status;
    }

    bool Parser::read_byte(const uint8_t& byte)
    {
        current_byte = byte;
        if(parser_state < kGotPayload)
        {
            ubx_message.buffer[message_length] = current_byte;
        }
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
                ubx_message.message_class = current_byte;
                check_message_class();
                break;

            case kGotMessageClass:
                ubx_message.message_id = current_byte;
                check_message_class_id();
                break;

            case kGotMessageID:
                advance();
                ubx_message.payload_length = current_byte;
                break;

            case kGotLength_1:
                get_length_2();
                break;

            case kGotLength_2:
                receive_payload();
                break;

            case kGotPayload:
                valid = advance_or_reset(ubx_message.get_checksum_a()==current_byte);
                break;
            
            case kGotChecksumA:
                valid = advance_or_reset(ubx_message.get_checksum_b()==current_byte);
                finish_message();
                break;
        }
        return valid;
    }
}