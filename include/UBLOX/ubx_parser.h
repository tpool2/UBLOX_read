#ifndef UBX_PARSER_H
#define UBX_PARSER_H
#include <cstdint>
#include <memory>
#include "UBLOX/ubx_defs.h"
#include "UBLOX/ubx_database.h"

using std::uint8_t;

namespace ublox::ubx
{
    class Parser
    {
        private:
            int parser_state = 0;
            int kByteSize = 8;
            uint8_t current_byte = 0;
            uint8_t message_class = 0x00;
            uint8_t message_id = 0x00;
            uint16_t message_length = 0x0000;
            std::unique_ptr<DatabaseInterface> database;
            void get_start_byte_1();
            void get_start_byte_2();
            void get_length_2();
            void advance_or_reset(bool advance);
            void advance();
            void reset();
            void check_message_class_id();
        
        public:
            Parser()
            {
                database = std::make_unique<Database>();
            };
            ~Parser()
            {

            };
            int get_parser_state() const;

            void read_byte(const uint8_t&);

            enum
            {
                kReset,
                kGotStartByte_1,
                kGotStartByte_2,
                kGotMessageClass,
                kGotMessageID,
                kGotLength_1,
                kGotLength_2,
            };
    };
}

#endif