#include "GNSS/satellite.hpp"

using bit_utils::get_msb_bits;

namespace gnss
{

constellation_t Satellite::get_constellation() const
{
    return constellation;
}

int SatelliteDatabase::update_satellite(int constellation, int satellite)
{
    return 0;
}

void SatelliteDatabase::update(const ublox::ubx::UBX_message_t& message)
{
    auto sfrbx = message.payload.RXM_SFRBX;
    switch(sfrbx.gnssId)
    {
        case kGPS:
            std::cout<<"GPS message"<<std::endl;
            if(gps::lnav::check_parity(sfrbx.dwrd))
            {
                std::cout<<"LNAV Message"<<std::endl;
            }
            break;
    }
}

namespace gps
{

}

}