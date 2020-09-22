#ifndef UBX_PARSER_H
#define UBX_PARSER_H
#include "UBLOX/ubx_defs.h"

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
    };
}

#endif