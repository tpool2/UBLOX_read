#include "Configuration/configuration.hpp"

namespace ublox { namespace configure
{

double seconds_elapsed(const clock_t &start_time)
{
    return static_cast<double>(clock()-start_time)/CLOCKS_PER_SEC;
}

bool val_set(std::shared_ptr<Ublox> my_ublox, uint32_t cfg_key, uint64_t cfg_data, uint8_t layer)
{   
    using bit_utils::get_lsb_bits;
    bool got_ack_ack = false;
    bool got_ack_nack = false;

    my_ublox->parser.register_callback(ubx::kCLASS_ACK, ubx::kACK_ACK, [&got_ack_ack](const ubx::UBX_message_t &ubx_msg)
    {
        std::cout<<"ACK_ACK"<<std::endl;
        got_ack_ack = true;
    });
    my_ublox->parser.register_callback(ubx::kCLASS_ACK, ubx::kACK_NACK, [&got_ack_nack](const ubx::UBX_message_t &ubx_msg)
    {
        std::cout<<"ACK_NACK"<<std::endl;
        got_ack_nack = true;
    });
    uint8_t payload_length = 9;
    uint8_t payload[payload_length] = {  ubx::CFG_VALSET_t::kVERSION_0,
                            layer,
                            0,
                            0,
                            get_lsb_bits<uint8_t>(&cfg_key, 0, 8),
                            get_lsb_bits<uint8_t>(&cfg_key, 8, 8),
                            get_lsb_bits<uint8_t>(&cfg_key, 16, 8),
                            get_lsb_bits<uint8_t>(&cfg_key, 24, 8),
                            get_lsb_bits<uint8_t>(&cfg_data, 0, 1) };
    ubx::UBX_message_t out_msg = ubx::create_message(ubx::kCLASS_CFG, ubx::kCFG_VALSET, payload_length, payload);
    my_ublox->comm->send_bytes(out_msg.buffer, 8+payload_length);

    for(int index = 0; index < 8+payload_length; ++index)
    {
        std::cout<<"Sent " << uint16_t(out_msg.buffer[index]) << std::endl;
    }

    clock_t start = clock();
    while(seconds_elapsed(start) < 10 && !got_ack_nack && !got_ack_ack);
    
    my_ublox->parser.pop_callback();
    my_ublox->parser.pop_callback();
    return got_ack_ack;
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
    bool got_ack = false;

    parser.register_callback(ublox::ubx::kCLASS_CFG, ublox::ubx::kCFG_VALGET, [&got_msg](const ublox::ubx::UBX_message_t& ubx_msg)
    {
        std::cout<<ubx_msg.payload.CFG_VALGET.version<<std::endl;
        got_msg = true;
    });

    parser.register_callback(ubx::kCLASS_ACK, ubx::kACK_NACK, [&got_ack](const ubx::UBX_message_t &response)
    {
        got_ack = true;
        std::cout<<"ACK_NACK"<<std::endl;
    });

    parser.register_callback(ubx::kCLASS_ACK, ubx::kACK_ACK, [&got_ack](const ubx::UBX_message_t &response)
    {
        got_ack = true;
        std::cout<<"ACK_ACK"<<std::endl;
    });

    serial->register_receive_callback([&parser](const uint8_t* buffer, size_t length)
    {
        parser.read_bytes(buffer, length);
    });

    auto request = cfg_val_get_message(ubx::CFG_VALGET_t::kSIGNAL_GPS);
    int length = sizeof(ubx::CFG_VALGET_t::request_t)+8;

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    serial->send_bytes(request.buffer, length);

    clock_t start = clock();
    while(!got_msg && seconds_elapsed(start) < 5);

    // Deregister callbacks
    parser.pop_callback();
    parser.pop_callback();
    parser.pop_callback();

    serial->register_receive_callback(nullptr);

    return got_msg;
}

} // namespace configure
} // namespace ublox