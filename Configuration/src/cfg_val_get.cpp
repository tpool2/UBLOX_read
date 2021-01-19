#include <string>
#include <iostream>
#include "async_comm/serial.h"
#include "Configuration/configure.h"

int main(int argc, char* argv[])
{
    std::string serial_port = "/dev/ttyACM0";
    if(argc > 1)
    {
        serial_port = argv[1];
    }
    std::cout<<serial_port<<std::endl;
    std::shared_ptr<async_comm::Serial> serial = std::make_shared<async_comm::Serial>(serial_port, 460800);
    std::shared_ptr<ublox::ubx::Parser> parser = std::make_shared<ublox::ubx::Parser>();
    serial->register_receive_callback([parser](const uint8_t* buffer, size_t length)
    {
        parser->read_bytes(buffer, length);
    });

    if(!serial->init())
    {
        std::cout<<"Failed to initialize serial port"<<std::endl;
        return 1;
    };
    bool result = ublox::configure::val_get(serial);

    serial->close();

    std::cout<<result<<std::endl;
    return 0;
}