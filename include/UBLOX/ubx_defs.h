#ifndef UBX_DEFS_H
#define UBX_DEFS_H
#include <cstdint>

using std::uint8_t;

namespace ublox::ubx
{
    enum
    {
        kStartByte_1 = 0xb5,
        kStartByte_2 = 0x62
    };

    enum
    {
        kCLASS_INF = 0x04,
        kCLASS_CFG = 0x06,
        kCLASS_UPD = 0x09,
        kCLASS_MON = 0x0a,
        kCLASS_TIM = 0x0d,
        kCLASS_LOG = 0x21,
        kCLASS_SEC = 0x27,
    };

    enum ACK
    {
        kCLASS_ACK = 0x05,
        kACK_ACK = 0x01,
        kACK_NACK = 0x00,
    };

    enum MGA
    {
        kCLASS_MGA = 0x13,
        kMGA_ACK = 0x60,
        kMGA_BDS = 0x03,
        kMGA_DBD = 0x80,
        kMGA_GAL = 0x02,
        kMGA_GLO = 0x06,
        kMGA_GPS = 0x00,
        kMGA_INI = 0x40,
        kMGA_QZSS = 0x05,
    };

    enum NAV
    {
        kCLASS_NAV = 0x01,
        kNAV_ORB = 0x34,
    };

    enum RXM
    {
        kCLASS_RXM = 0x02,
        kRXM_MEASX = 0x14,
        kRXM_RAWX = 0x15,
        kRXM_SFRBX = 0x13,
    };
}
#endif