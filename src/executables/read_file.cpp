#include <iostream>
#include <fstream>
#include <cstdint>
#include "UBLOX/ubx_parser.h"
#include "UBLOX/ubx_sfrbx_parser.h"

int main(int argc, char* argv[])
{
    std::ifstream ifs;
    ifs.open(argv[1]);

    ublox::ubx::Parser parser;
    ublox::ubx::SFRBX_Parser nav_parser;
    parser.add_message_parser(std::make_shared<ublox::ubx::SFRBX_Parser>(nav_parser));
    uint8_t byte;

    while(ifs.good())
    {
        byte = ifs.get();
        // std::cout<<int(byte)<<std::endl;
        parser.read_byte((byte));
    }
    ifs.close();

    return 0;
}