#include <string>
#include <iostream>
#include "async_comm/serial.h"
#include "Configuration/configure.h"
#include "ublox/ublox.hpp"

int main(int argc, char* argv[])
{
    std::string serial_port = "/dev/ttyACM0";
    uint32_t configuration_key = ublox::ubx::CFG_VALSET_t::kSIGNAL_GPS_L2;
    if(argc > 1)
    {
        serial_port = argv[1];
    }
    if(argc > 2)
    {
        configuration_key = static_cast<uint32_t>(std::stoi(argv[2]));
    }
    std::shared_ptr<ublox::Ublox> my_ublox = std::make_shared<ublox::Ublox>(serial_port);

    uint8_t result = ublox::configure::val_get<uint8_t>(my_ublox, configuration_key);
    std::cout<<static_cast<uint16_t>(result)<<std::endl;
    return 0;
}