#include "Configuration/configuration.hpp"

namespace ublox { namespace configure
{

double seconds_elapsed(const clock_t &start_time)
{
    return static_cast<double>(clock()-start_time)/CLOCKS_PER_SEC;
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

bool configure_stationary_base(std::shared_ptr<Ublox> my_ublox)
{
    return toggle_moving_base(my_ublox, false)
        && toggle_stationary_base(my_ublox, true);
}

bool toggle_moving_base(std::shared_ptr<Ublox> my_ublox, bool on)
{
    return val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_4072_0USB, static_cast<uint8_t>(1*on))
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_4072_1USB, static_cast<uint8_t>(1*on))
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1077USB, static_cast<uint8_t>(1*on))
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1087USB, static_cast<uint8_t>(1*on))
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1097USB, static_cast<uint8_t>(1*on))
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1127USB, static_cast<uint8_t>(1*on))
        && val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1230USB, static_cast<uint8_t>(1*on));
}

bool toggle_stationary_base(std::shared_ptr<Ublox> my_ublox, bool on)
{
    return val_set(my_ublox, ubx::CFG_VALSET_t::kRTCM_1005USB, static_cast<uint8_t>(1*on))
        && val_set(my_ublox, ubx::CFG_VALSET_t::kMSGOUT_SVIN, static_cast<uint8_t>(1*on))
        && val_set(my_ublox, ubx::CFG_VALSET_t::kTMODE_MODE, static_cast<uint8_t>(1*on))
        && val_set(my_ublox, ubx::CFG_VALSET_t::kTMODE_SVIN_ACC_LIMIT, static_cast<uint32_t>(100*on))
        && val_set(my_ublox, ubx::CFG_VALSET_t::kTMODE_SVIN_MIN_DUR, static_cast<uint32_t>(60*on));
}

} // namespace configure
} // namespace ublox