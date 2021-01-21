#ifndef CONFIGURE_H
#define CONFIGURE_H
#include <chrono>
#include <memory>
#include "async_comm/serial.h"
#include "UBX/ubx.hpp"
#include "ublox/ublox.hpp"
#include "bit_utils/bit_utils.h"

namespace ublox { namespace configure
{

double seconds_elapsed(const clock_t&);

template <class T> ubx::UBX_message_t cfg_val_set_message(uint32_t configuration_key, T configuration_data, uint8_t layer = ubx::CFG_VALSET_t::kRAM)
{
    using bit_utils::get_lsb_bits;
    
    uint16_t payload_length = 8 + sizeof(T);
    uint8_t payload[payload_length];
    memset(payload, 0, payload_length);

    payload[0] = ubx::CFG_VALSET_t::kVERSION_0;
    payload[1] = layer;
    payload[4] = get_lsb_bits<uint8_t>(&configuration_key, 0, 8);
    payload[5] = get_lsb_bits<uint8_t>(&configuration_key, 8, 8);
    payload[6] = get_lsb_bits<uint8_t>(&configuration_key, 16, 8);
    payload[7] = get_lsb_bits<uint8_t>(&configuration_key, 24, 8);
    for(int index = 8; index < payload_length; ++index)
    {
        payload[index] = get_lsb_bits<uint8_t>(&configuration_data, 8*(index-8), 8);
    }

    return ubx::create_message(ubx::kCLASS_CFG, ubx::kCFG_VALSET, payload_length, payload);
}
ubx::UBX_message_t cfg_val_get_message(uint32_t configuration_key);
bool configure_moving_base(std::shared_ptr<Ublox> my_ublox);
bool configure_stationary_base(std::shared_ptr<Ublox> my_ublox);
bool toggle_moving_base(std::shared_ptr<Ublox> my_ublox, bool on=true);
bool toggle_stationary_base(std::shared_ptr<Ublox> my_ublox, bool on=true);
template <class T> T val_get(std::shared_ptr<Ublox> my_ublox, uint32_t configuration_key)
{
    bool got_configuration_data = false;
    bool got_ack_ack = false;
    bool got_ack_nack = false;
    T configuration_data = 0;

    my_ublox->parser.register_callback(ubx::kCLASS_CFG, ubx::kCFG_VALGET, [&got_configuration_data, &configuration_data, configuration_key](const ublox::ubx::UBX_message_t& ubx_msg)
    {
        got_configuration_data = true;
        if(configuration_key == bit_utils::get_lsb_bits<uint32_t>(ubx_msg.payload.buffer, 4))
        {
            std::cout << "Got good response" << std::endl;
            configuration_data = bit_utils::get_lsb_bits<T>(ubx_msg.payload.buffer, 8);
        }
        else
        {
            std::cout << "Got bad response" << std::endl;
        }
    });
    my_ublox->parser.register_callback(ubx::kCLASS_ACK, ubx::kACK_ACK, [&got_ack_ack](const ubx::UBX_message_t &ubx_msg)
    {
        got_ack_ack = true;
        std::cout<<"ACK_ACK"<<std::endl;
    });
    my_ublox->parser.register_callback(ubx::kCLASS_ACK, ubx::kACK_NACK, [&got_ack_nack](const ubx::UBX_message_t &ubx_msg)
    {
        got_ack_nack = true;
        std::cout<<"ACK_NACK"<<std::endl;
    });

    ubx::UBX_message_t out_message = cfg_val_get_message(configuration_key);
    my_ublox->comm->send_bytes(out_message.buffer, 8+out_message.payload_length);

    clock_t start = clock();
    while(seconds_elapsed(start) < 2 && !(got_ack_ack && got_configuration_data) && !got_ack_nack);

    my_ublox->parser.pop_callbacks(3);

    return configuration_data;
}

template <class T> bool val_set(std::shared_ptr<Ublox> my_ublox, uint32_t cfg_key, T cfg_data, uint8_t layer=ubx::CFG_VALSET_t::kRAM)
{
    bool got_ack_ack = false;
    bool got_ack_nack = false;

    my_ublox->parser.register_callback(ubx::kCLASS_ACK, ubx::kACK_ACK, [&got_ack_ack, cfg_key, cfg_data](const ubx::UBX_message_t &ubx_msg)
    {
        std::cout << "ACK_ACK: " << cfg_key << " : " << static_cast<uint16_t>(cfg_data) << std::endl;
        got_ack_ack = true;
    });
    my_ublox->parser.register_callback(ubx::kCLASS_ACK, ubx::kACK_NACK, [&got_ack_nack, cfg_key, cfg_data](const ubx::UBX_message_t &ubx_msg)
    {
        std::cout << "ACK_NACK: " << cfg_key << " : " << static_cast<uint16_t>(cfg_data) << std::endl;
        got_ack_nack = true;
    });
    
    ubx::UBX_message_t out_msg = cfg_val_set_message(cfg_key, cfg_data);
    my_ublox->comm->send_bytes(out_msg.buffer, 8+out_msg.payload_length);

    clock_t start = clock();
    while(seconds_elapsed(start) < 2 && !got_ack_nack && !got_ack_ack);
    
    my_ublox->parser.pop_callbacks(2);
    return got_ack_ack;
}
} // namespace configure
} // namespace ublox

#endif