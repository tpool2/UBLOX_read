#ifndef UBX_PARSER_H
#define UBX_PARSER_H
#include <cstdint>
#include <memory>
#include <iostream>
#include <string.h>
#include <vector>
#include <functional>
#include "UBLOX/ubx_defs.h"
#include "UBLOX/ubx_database.h"

using std::uint8_t;

namespace ublox::ubx
{
    class Parser
    {
        private:
            class Callback
            {
                public:
                    Callback(uint8_t message_class, uint8_t message_id, std::function<void(const uint8_t* payload, size_t size)> function)
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
                    std::function<void(const uint8_t* payload, size_t size)> callback_function;
            };

            union
            {
                uint8_t buffer[ublox::ubx::BUFFER_SIZE];
                struct
                {
                    uint8_t start_byte_1    : 8;
                    uint8_t start_byte_2    : 8;
                    uint8_t message_class   : 8;
                    uint8_t message_id      : 8;
                    uint16_t payload_length : 16;
                    UBX_message_t payload;
                };
            };
            

            std::vector<Callback> callbacks;
            int parser_state = 0;
            int kByteSize = 8;
            uint8_t checksum_a = 0;
            uint8_t checksum_b = 0;
            bool valid = false;
            uint8_t current_byte = 0;
            int message_length = 0;
            std::unique_ptr<DatabaseInterface> database;
            void get_start_byte_1();
            void get_start_byte_2();
            void get_length_2();
            bool advance_or_reset(bool advance);
            void advance();
            void reset();
            void check_message_class();
            void check_message_class_id();
            void receive_payload();
            void calculate_checksums();
            void finish_message();
        
        public:
            Parser()
            {
                database = std::make_unique<Database>();
                reset();
            };
            ~Parser()
            {

            };
            int get_parser_state() const;
            void register_callback(uint8_t message_id, uint8_t message_class, std::function<void(const uint8_t *payload, size_t length)>);
            bool read_byte(const uint8_t& byte);

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

#endif