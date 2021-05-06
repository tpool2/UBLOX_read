#include "GNSS/satellite.hpp"

using bit_utils::get_msb_bits;

namespace gnss
{

constellation_t Satellite::get_constellation() const
{
    return constellation;
}

int Satellite::get_id() const
{
    return id;
}

bool Satellite::update(uint32_t* words)
{
    return true;
}

void SatelliteDatabase::update(const ublox::ubx::RXM_SFRBX_t& sfrbx)
{
    switch(sfrbx.gnssId)
    {
        case kGPS:
            if(gps::lnav::check_parity(sfrbx.dwrd)
            {
                if(constellation_map[kGPS].count(sfrbx.svId) == 0)
                {
                    constellation_map[kGPS][sfrbx.svId] = std::make_shared<gps::GPS_Satellite>(sfrbx.svId);
                }
                constellation_map[kGPS][sfrbx.svId]->update(sfrbx.dwrd);
            }
            else if(bit_utils::get_msb_bits<uint8_t>(sfrbx.dwrd, 0, 8) == gps::kPreamble)
            {

            }
            break;
    }
}

namespace gps
{
    bool GPS_Satellite::update(uint32_t* words)
    {
        return true;
    }
}

}