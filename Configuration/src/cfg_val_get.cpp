#include <string>
#include "async_comm/serial.h"
#include "Configuration/configure.h"

int main(int argc, char* argv[])
{
    std::string serial_port = "/dev/ttyACM0";
    if(argc > 1)
    {
        serial_port = argv[1];
    }
    std::shared_ptr<async_comm::Serial> serial = std::make_shared<async_comm::Serial>(serial_port, 460800);
    bool result = ublox::configure::val_get(serial);
    std::cout<<result<<std::endl;
    return 0;
}