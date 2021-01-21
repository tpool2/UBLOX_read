#include "Configuration/configuration.hpp"

namespace ublox { namespace configure
{

double seconds_elapsed(const clock_t &start_time)
{
    return static_cast<double>(clock()-start_time)/CLOCKS_PER_SEC;
}

bool val_set(std::shared_ptr<Ublox> my_ublox, uint32_t cfg_key, uint8_t cfg_data, uint8_t layer)
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
    
    ubx::UBX_message_t out_msg = cfg_val_set_message(cfg_key, static_cast<uint8_t>(cfg_data));
    my_ublox->comm->send_bytes(out_msg.buffer, 8+out_msg.payload_length);

    clock_t start = clock();
    while(seconds_elapsed(start) < 2 && !got_ack_nack && !got_ack_ack);
    
    my_ublox->parser.pop_callbacks(2);
    return got_ack_ack;
}

ubx::UBX_message_t cfg_val_get_message(uint32_t configuration_key)
{
    union
    {
        ubx::CFG_VALGET_t::request_t cfg_request;
        uint8_t payload[sizeof(cfg_request)];
    };
    
    cfg_request.version = ubx::CFG_VALGET_t::kREQUEST;
    cfg_request.layer = ubx::CFG_VALGET_t::kRAM;
    cfg_request.position = 0;
    cfg_request.cfgDataKey.keyID = configuration_key;

    auto message = ubx::create_message(ubx::kCLASS_CFG, ubx::kCFG_VALGET, sizeof(ubx::CFG_VALGET_t::request_t), payload);
    return message;
}

bool configure_moving_base(std::shared_ptr<Ublox> my_ublox)
{
    return toggle_stationary_base(my_ublox, false)
        && toggle_moving_base(my_ublox, true);
}

bool toggle_moving_base(std::shared_ptr<Ublox> my_ublox, bool on)
{
    return val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_4072_0USB, 1*on)
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_4072_1USB, 1*on)
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1077USB, 1*on)
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1087USB, 1*on)
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1097USB, 1*on)
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1127USB, 1*on)
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1230USB, 1*on);
}

bool toggle_stationary_base(std::shared_ptr<Ublox> my_ublox, bool on)
{
    return val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1005USB, 1*on)
        && val_set(my_ublox, ubx::CFG_VALSET_t::kMSGOUT_SVIN, 1*on)
        && val_set(my_ublox, ubx::CFG_VALSET_t::kTMODE_MODE, 1*on);
        // && val_set(my_ublox, ubx::CFG_VALSET_t::kTMODE_SVIN_ACC_LIMIT, 100*on)
        // && val_set(my_ublox, ubx::CFG_VALSET_t::kTMODE_SVIN_MIN_DUR, 60*on);
}

} // namespace configure
} // namespace ublox