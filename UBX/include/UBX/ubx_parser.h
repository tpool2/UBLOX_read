#ifndef UBX_PARSER_H
#define UBX_PARSER_H
#include <cstdint>
#include <memory>
#include <iostream>
#include <string.h>
#include <vector>
#include <functional>
#include "UBX/ubx_defs.h"
#include "UBX/ubx_database.h"
#include "UBX/ubx_message.h"

using std::uint8_t;

namespace ublox
{
    namespace ubx
    {
    class Parser
    {
        private:
            class Callback
            {
                public:
                    Callback(uint8_t message_class, uint8_t message_id, std::function<void(const UBX_message_t&)> function)
                    {
                        this->message_class = message_class;
                        this->message_id = message_id;
                        this->callback_function = function;
                    }
                    bool matches(uint8_t _message_class, uint8_t _message_id)
                    {
                        return message_class == _message_class && message_id == _message_id;
                    }
                    uint8_t message_class = 0x00;
                    uint8_t message_id = 0x00;
                    ubx_callback_function callback_function;
            };

            UBX_message_t ubx_message;
            
            int parser_state = 0;
            int kByteSize = 8;
            int message_length = 0;
            bool valid = false;
            uint8_t current_byte = 0;
            std::unique_ptr<DatabaseInterface> database;
            std::vector<Callback> callbacks;

            void get_start_byte_1();
            void get_start_byte_2();
            void get_length_2();
            bool advance_or_reset(bool advance);
            void advance();
            void reset();
            void check_message_class();
            void check_message_class_id();
            void receive_payload();
            void finish_message();
        
        public:
            Parser()
            {
                database = std::unique_ptr<Database>(new Database());
                reset();
            };
            ~Parser()
            {

            };
            void pop_callbacks(int number_of_callbacks);
            int get_parser_state() const;
            void register_callback(uint8_t message_class, uint8_t message_id, std::function<void(const UBX_message_t&)>);
            bool read_byte(const uint8_t& byte);
            bool read_bytes(const uint8_t* buffer, size_t length);

            enum
            {
                kReset,
                kGotStartByte_1,
                kGotStartByte_2,
                kGotMessageClass,
                kGotMessageID,
                kGotLength_1,
                kGotLength_2,
                kGotPayload,
                kGotChecksumA,
                kGotChecksumB,
            };
    };
    }
}

#endif