#include "Configuration/configuration.hpp"

namespace ublox { namespace configure
{

double seconds_elapsed(const clock_t &start_time)
{
    return static_cast<double>(clock()-start_time)/CLOCKS_PER_SEC;
}

bool val_set(std::shared_ptr<async_comm::Serial> serial)
{   
    clock_t start = clock();
    while(seconds_elapsed(start) < 5);

    return true;
}

ubx::UBX_message_t cfg_val_get_message(uint32_t configuration_key)
{
    union
    {
        ubx::CFG_VALGET_t::request_t cfg_request;
        uint8_t payload[sizeof(cfg_request)];
    };
    
    cfg_request.version = ubx::CFG_VALGET_t::kREQUEST;
    cfg_request.layer = ubx::CFG_VALGET_t::kRAM;
    cfg_request.position = 0;
    cfg_request.cfgDataKey.keyID = configuration_key;

    auto message = ubx::create_message(ubx::kCLASS_CFG, ubx::kCFG_VALGET, sizeof(ubx::CFG_VALGET_t::request_t), payload);
    return message;
}

bool val_get(std::shared_ptr<async_comm::Serial> serial)
{
    ublox::ubx::Parser parser;
    bool got_msg = false;

    parser.register_callback(ublox::ubx::kCLASS_CFG, ublox::ubx::kCFG_VALGET, [&got_msg](const ublox::ubx::UBX_message_t& ubx_msg)
    {
        std::cout<<"Got it!"<<std::endl;
        got_msg = true;
    });

    parser.register_callback(ubx::kCLASS_ACK, ubx::kACK_NACK, [](const ubx::UBX_message_t &response)
    {
        std::cout<<"Not Acknowledged"<<std::endl;
    });

    parser.register_callback(ubx::kCLASS_ACK, ubx::kACK_ACK, [](const ubx::UBX_message_t &response)
    {
        std::cout<<"Acknowledged"<<std::endl;
    });

    serial->register_receive_callback([&parser](const uint8_t* buffer, size_t length)
    {
        parser.read_bytes(buffer, length);
    });

    auto request = cfg_val_get_message(ubx::CFG_VALGET_t::kSIGNAL_GPS);
    int length = sizeof(ubx::CFG_VALGET_t::request_t)+8;
    serial->send_bytes(request.buffer, length);

    clock_t start = clock();
    while(!got_msg && seconds_elapsed(start) < 5);

    return got_msg;
}

} // namespace configure
} // namespace ublox