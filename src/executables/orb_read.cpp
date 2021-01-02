#include <iostream>
#include <fstream>
#include <cstdint>
#include "UBX/ubx.hpp"
#include "GNSS/gnss.hpp"

int main(int argc, char* argv[])
{
    std::ifstream ifs;
    ifs.open(argv[1]);

    ublox::ubx::Parser parser;
    
    parser.register_callback(ublox::ubx::kCLASS_NAV, ublox::ubx::kNAV_ORB, [](const ublox::ubx::UBX_message_t& message)
    {
        std::cout<<"Nav ORB message"<<std::endl;
    });
    uint8_t byte;

    while(ifs.good())
    {
        byte = ifs.get();
        parser.read_byte((byte));
    }
    ifs.close();

    return 0;
}