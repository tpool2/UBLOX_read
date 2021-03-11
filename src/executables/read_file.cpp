#include <iostream>
#include <fstream>
#include <cstdint>
#include <ctime>
#include "UBX/ubx.hpp"
#include "GNSS/gnss.hpp"

int main(int argc, char* argv[])
{
    std::ifstream ifs;
    ifs.open(argv[1]);

    ublox::ubx::Parser parser;
    gnss::SatelliteDatabase database;
    
    parser.register_callback(ublox::ubx::kCLASS_RXM, ublox::ubx::kRXM_SFRBX, [&database](const ublox::ubx::UBX_message_t& message)
    {
        database.update(message);
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