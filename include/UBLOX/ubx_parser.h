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
            uint8_t checksum_a = 0;
            uint8_t checksum_b = 0;
            uint16_t payload_bytes_received = 0;
            void update_checksum();
            std::unique_ptr<DatabaseInterface> database;
            bool get_start_byte_1();
            bool get_start_byte_2();
            bool get_length_2();
            bool advance_or_reset(bool advance);
            void advance();
            void reset();
            bool check_message_class_id();
            bool get_payload();
            bool verify_checksum_a();
            bool verify_checksum_b();
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

            bool read_byte(const uint8_t&);
            uint8_t get_checksum_a() const;

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