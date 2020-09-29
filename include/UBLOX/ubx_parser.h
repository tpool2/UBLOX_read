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
            uint8_t message_class = 0x00;
            uint8_t message_id = 0x00;
            std::unique_ptr<DatabaseInterface> database;
        
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
            };
    };
}

#endif