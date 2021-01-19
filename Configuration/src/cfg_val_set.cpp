#include <string>
#include "async_comm/serial.h"
#include "Configuration/configure.h"
#include "ublox/ublox.hpp"

int main(int argc, char* argv[])
{
    std::string serial_port = "/dev/ttyACM0";
    if(argc > 1)
    {
        serial_port = argv[1];
    }
    std::shared_ptr<ublox::Ublox> my_ublox = std::make_shared<ublox::Ublox>(serial_port, 460800);
    bool result = ublox::configure::val_set(my_ublox, ublox::ubx::CFG_VALSET_t::kSIGNAL_GPS_L2, 1);
    std::cout<<result<<std::endl;
    return 0;
}