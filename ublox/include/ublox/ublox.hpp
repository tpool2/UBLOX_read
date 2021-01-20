#ifndef UBLOX_HPP
#define UBLOX_HPP
#include <iostream>
#include "async_comm/serial.h"
#include "UBX/ubx.hpp"

namespace ublox
{
class Ublox
{
    public:
        std::shared_ptr<async_comm::Comm> comm;
        ubx::Parser parser;

        Ublox(std::string serial_port, unsigned int baud_rate=460800)
        {
            comm = std::make_shared<async_comm::Serial>(serial_port, baud_rate);
            comm->register_receive_callback([this](const uint8_t* buffer, size_t length)
            {
                parser.read_bytes(buffer, length);
            });
            if(!comm->init())
            {
                std::cout<<"Failed to initialize serial port at "<<serial_port<<std::endl;
            }
        }
        ~Ublox()
        {
            comm->close();
        }
};
} // namespace ublox
#endif