#include "ublox/ublox.hpp"

namespace ublox
{

void Ublox::log_bytes_to_file(std::filesystem::path log_path)
{
    if(log_stream.is_open())
    {
        log_stream.close();
    }
    // std::time_t current_time;
    // struct std::tm * ptm;
    // std::time(&current_time);
    // ptm = std::gmtime(&current_time);


    log_stream.open(log_path, std::ios::out | std::ios::binary | std::ios::trunc);
    comm->register_receive_callback([this](const uint8_t* buffer, size_t length)
    {
        log_stream.write((const char*)(buffer), length);
        parser.read_bytes(buffer, length);
    });
}

} // namespace ublox