#ifndef SATELLITE_HPP
#define SATELLITE_HPP

#include <memory>
#include <iostream>
#include <map>
#include <cstdint>

#include "UBX/ubx.hpp"

#include "bit_utils/bit_utils.h"
#include "GNSS/ephemeris.hpp"
#include "GNSS/gps_lnav.hpp"

namespace gnss
{

typedef enum
{
    kGPS,
    kSBAS,
    kGALILEO,
    kBEIDOU,
    kIMES,
    kQZSS,
    kGLONASS,
} constellation_t;

class Satellite
{
    public:
        constellation_t get_constellation() const;
        virtual bool update(uint32_t* words);
        int get_id() const;
        Satellite()
        {
            id = 0;
        }
        Satellite(int sat_id)
        {
            id = sat_id;
        }
    
    protected:
        constellation_t constellation;
        int id;
        std::shared_ptr<EphemerisInterface> ephemeris;
};

class SatelliteDatabase
{
    private:
        typedef std::map<int, std::shared_ptr<Satellite> > SatelliteMap;
        SatelliteMap GPS_Satellites;
        std::map<int, SatelliteMap> constellation_map
        {
            {kGPS, GPS_Satellites},
        };

    public:
        void update(const ublox::ubx::UBX_message_t& message);
};

namespace gps
{

class GPS_Satellite: public Satellite
{
    public:
        GPS_Satellite(int sat_id)
        {
            constellation = kGPS;
        };
        bool update(uint32_t* words);
};

}

namespace galileo
{

class Galileo_Satellite: public Satellite
{
    public:
        Galileo_Satellite(int sat_id)
        {
            constellation = kGALILEO;
        };
};

}

}

#endif