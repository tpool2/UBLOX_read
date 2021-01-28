#ifndef SATELLITE_HPP
#define SATELLITE_HPP

#include <memory>
#include <iostream>
#include <map>

#include "GNSS/ephemeris.hpp"

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
        virtual bool update_ephemeris(std::shared_ptr<EphemerisInterface>);
    
    protected:
        constellation_t constellation;
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
        int update_satellite(int constellation, int satellite);
};

namespace gps
{

class GPS_Satellite: public Satellite
{
    public:
        GPS_Satellite(bool cnav = true)
        {
            constellation = kGPS;
            if(cnav)
            {
                ephemeris = std::make_shared<CNAVEphemeris>();
            }
            else
            {
                ephemeris = std::make_shared<L1CAEphemeris>();
            } 
        };
        bool update_ephemeris(std::shared_ptr<EphemerisInterface>) override;
};

}

namespace galileo
{

class Galileo_Satellite: public Satellite
{
    public:
        Galileo_Satellite()
        {
            constellation = kGALILEO;
        };
        bool update_ephemeris(std::shared_ptr<EphemerisInterface>) override;
};

}

}

#endif