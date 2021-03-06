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

void SatelliteDatabase::update(const ublox::ubx::UBX_message_t& message)
{
    auto sfrbx = message.payload.RXM_SFRBX;
    switch(sfrbx.gnssId)
    {
        case kGPS:
            if(gps::lnav::check_parity(sfrbx.dwrd))
            {
                if(constellation_map[kGPS].count(sfrbx.svId) == 0)
                {
                    constellation_map[kGPS][sfrbx.svId] = std::make_shared<gps::GPS_Satellite>(sfrbx.svId);
                }
                std::cout<<"SV ID: "<<uint16_t(sfrbx.svId)<<std::endl;
                std::cout<<"Subframe ID: " << uint16_t(gps::lnav::get_bits<uint8_t>(&sfrbx.dwrd[1], 19, 3)) << std::endl;
                if(sfrbx.svId == 07)
                {
                    for(int i = 0; i < gps::kWordsPerSubframe; ++i)
                    {
                        std::cout << std::hex << sfrbx.dwrd[i]<<std::dec<<std::endl;
                    }
                }
                constellation_map[kGPS][sfrbx.svId]->update(sfrbx.dwrd);
            }
            else if(bit_utils::get_msb_bits<uint8_t>(sfrbx.dwrd, 0, 8) == gps::kPreamble)
            {
                // std::cout<<"CNAV Message"<<std::endl;
                // std::cout<<uint16_t(sfrbx.svId)<<std::endl;
                // for(int i = 0; i < gps::kWordsPerSubframe; ++i)
                // {
                    // std::cout << std::hex << sfrbx.dwrd[i]<<std::dec<<std::endl;
                // }
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