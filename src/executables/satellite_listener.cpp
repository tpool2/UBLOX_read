#include <iostream>
#include "ublox/ublox.hpp"
#include "GNSS/gnss.hpp"
#include "Configuration/configuration.hpp"

int main(int argc, char* argv[])
{
    std::string serial_port = "/dev/ttyACM0";
    if(argc > 1)
    {
        serial_port = argv[1];
    }

    std::shared_ptr<ublox::Ublox> my_ublox = std::make_shared<ublox::Ublox>(serial_port);
    my_ublox->log_bytes_to_file("satellite_log");
    ublox::configure::configure_moving_base(my_ublox);
    ublox::configure::val_set(my_ublox, ublox::ubx::CFG_VALSET_t::kSIGNAL_GPS, static_cast<uint8_t>(1));
    ublox::configure::val_set(my_ublox, ublox::ubx::CFG_VALSET_t::kSIGNAL_GPS_L1, static_cast<uint8_t>(1));
    ublox::configure::val_set(my_ublox, ublox::ubx::CFG_VALSET_t::kSIGNAL_GPS_L2, static_cast<uint8_t>(1));
    ublox::configure::val_set(my_ublox, ublox::ubx::CFG_VALSET_t::kMSGOUT_SFRBX, static_cast<uint8_t>(1));
    my_ublox->parser.register_callback(ublox::ubx::kCLASS_RXM, ublox::ubx::kRXM_SFRBX, [](const ublox::ubx::UBX_message_t &ubx_msg)
    {
        std::cout<<"Message"<<std::endl;
    });

    while(true);
    return 0;
}