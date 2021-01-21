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
    std::cout<<ublox::configure::configure_moving_base(my_ublox)<<std::endl;
    return 0;
}