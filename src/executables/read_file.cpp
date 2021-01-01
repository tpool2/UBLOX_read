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
    gnss::NavParser sfrbx_parser;
    
    parser.register_callback(ublox::ubx::kCLASS_RXM, ublox::ubx::kRXM_SFRBX, [&sfrbx_parser](const ublox::ubx::UBX_message_t& message)
    {
        sfrbx_parser.parse_sfrbx(message);
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