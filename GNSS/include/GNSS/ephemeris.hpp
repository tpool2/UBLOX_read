#ifndef EPHEMERIS_HPP
#define EPHEMERIS_HPP
#include <cstdint>

namespace gnss
{

class EphemerisInterface
{
    protected:
        double x_ecef;
        double y_ecef;
        double z_ecef;
    
    public:
        virtual void update_location() = 0;
        double get_x_ecef() const;
        double get_y_ecef() const;
        double get_z_ecef() const;
};

namespace gps
{

class L2Ephemeris: public EphemerisInterface
{
    private:
        // Message 10
        uint16_t week_number;
        int8_t ED_accuracy;
        uint32_t CEI_time_of_week;
        double semi_major_axis_diff;
        double semi_major_axis_change_rate;
        double mean_motion_diff;
        double mean_motion_diff_rate;
        double mean_anomaly;
        double eccentricity;
        double argument_of_perigee;

        // Message 11
        uint32_t ephemeris_time_of_week;
        double longitude_orbit_plane;
        double rate_right_ascension_diff;
        double inclination_angle;
        double sin_inclination;
        double cos_inclination;
        double sin_orbit_radius;
        double cos_orbit_radius;
        double sin_latitude;
        double cos_latitude;

    public:
        void update_location() override;
};

class L1CAEphemeris
{
    private:
        // Subframe 2
        uint8_t IODE;
        double sin_orbit_radius;
};

}

}

#endif