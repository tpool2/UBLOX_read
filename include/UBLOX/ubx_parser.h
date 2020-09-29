#ifndef UBX_PARSER_H
#define UBX_PARSER_H
#include <cstdint>
#include "UBLOX/ubx_defs.h"

using std::uint8_t;

namespace ublox::ubx
{
    class Parser
    {
        private:
            int parser_state = 0;
        
        public:
            Parser()
            {

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