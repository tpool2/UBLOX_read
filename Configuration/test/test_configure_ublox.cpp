#include <gtest/gtest.h>
#include "Configuration/configuration.hpp"
#include "UBX/ubx.hpp"
#include "ublox/ublox.hpp"

using namespace ublox::ubx;

class MockUbloxReceiver: public ::testing::Test
{
    protected:
        Parser ublox_parser;
        Parser listener_parser;
        bool got_ack_cfg_val_get;


        void SetUp()
        {
            got_ack_cfg_val_get = false;
            ublox_parser.register_callback(kCLASS_CFG, kCFG_VALGET, [this](const UBX_message_t& ubx_message)
            {
                CFG_VALGET_t::request_t request = ubx_message.payload.CFG_VALGET;
                
                union
                {
                    ACK_ACK_t ack_response;
                    uint8_t ack_payload[2];
                };

                ack_response.clsID = kCLASS_CFG;
                ack_response.msgID = kCFG_VALGET;

                UBX_message_t ack_message = create_message(kCLASS_ACK, kACK_ACK, 2, ack_payload);
                send_bytes(ack_message.buffer, 8+sizeof(ACK_ACK_t), listener_parser);
            });

            listener_parser.register_callback(kCLASS_ACK, kACK_ACK, [this](const UBX_message_t& ubx_message)
            {
                ACK_ACK_t ack_msg = ubx_message.payload.ACK_ACK;
                if(ack_msg.clsID==kCLASS_CFG && ack_msg.msgID==kCFG_VALGET)
                {
                    got_ack_cfg_val_get = true;
                }
            });
        }

        void send_bytes(const uint8_t* buffer, size_t length, Parser &parser)
        {
            for(size_t position = 0; position < length; ++position)
            {
                parser.read_byte(buffer[position]);
            }
        }
};

TEST_F(MockUbloxReceiver, SendConfigurationRequest)
{
    auto message = ublox::configure::cfg_val_get_message(CFG_VALSET_t::kMSGOUT_SFRBX);
    send_bytes(message.buffer, sizeof(CFG_VALGET_t::request_t)+8, ublox_parser);
    ASSERT_TRUE(got_ack_cfg_val_get);
}