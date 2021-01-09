#ifndef SATELLITE_HPP
#define SATELLITE_HPP

#include <memory>
#include <iostream>
#include <map>

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
    
    protected:
        constellation_t constellation;
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
        GPS_Satellite()
        {
            constellation = kGPS;
        };
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
};

}

}

#endif