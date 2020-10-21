#ifndef UBX_CFG_VAL_SET
#define UBX_CFG_VAL_SET
#include "UBLOX/ubx_listener.h"

namespace ublox::ubx
{
    class ConfigurationSetter: public MessageParser
    {
        private:
        
        public:
            void read_byte(uint8_t byte) override;
            void reset() override;
            void finish_message() override;
            void set_message_rate(uint8_t message_class, uint8_t message_id, )
    };
}

#endif