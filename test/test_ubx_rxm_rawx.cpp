#include <gtest/gtest.h>
#include "UBLOX/ubx_rxm_rawx.h"
#include <memory>

using namespace ublox::ubx;

TEST(CreateRXM_RAWXParser, Test1)
{
    RXM_RAWX_Parser parser;
}

TEST(CreateRXM_RAWXParser, Pointer)
{
    std::shared_ptr<MessageParser> parser = std::make_shared<RXM_RAWX_Parser>();
}