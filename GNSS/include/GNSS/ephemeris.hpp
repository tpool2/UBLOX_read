#ifndef EPHEMERIS_HPP
#define EPHEMERIS_HPP
#include <cstdint>
#include <string>
#include <sstream>
#include "Bit_Utils/bit_utils.h"

using bit_utils::get_bits;

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
        virtual std::string to_string() const = 0;
};

namespace gps
{
class L2C_Message
{
    protected:
        uint8_t prn;
        uint8_t message_type_id;
        uint32_t message_tow;
        bool alert_flag;
};

class message_11: public L2C_Message
{
    public:
        message_11(const uint32_t* words)
        {
            prn = get_bits<uint8_t>(words, 8, 6);
            message_type_id = get_bits<uint8_t>(words, 14, 6);
            message_tow = get_bits<uint32_t>(words, 20, 17);
            alert_flag = get_bits<bool>(words, 37, 1);
            ephemeris_time_of_week = get_bits<uint16_t>(words, 38, 11);
            longitude_orbit_plane = pow(2,-32)*static_cast<double>(get_bits<int64_t>(words, 49, 33));
            inclination_angle = pow(2, -32)*static_cast<double>(get_bits<int64_t>(words, 82, 33));
            rate_right_ascension_diff = pow(2, -44)*static_cast<double>(get_bits<int32_t>(words, 115, 17));
            rate_inclination_angle = pow(2, -44)*static_cast<double>(get_bits<int16_t>(words, 15));
            sin_inclination = pow(2, -30)*static_cast<double>(get_bits<int16_t>(words, 147, 16));
            cos_inclination = pow(2, -30)*static_cast<double>(get_bits<int16_t>(words, 163, 16));
            sin_orbit_radius = pow(2, -8)*static_cast<double>(get_bits<int32_t>(words, 179, 24));
            cos_orbit_radius = pow(2, -8)*static_cast<double>(get_bits<int32_t>(words, 203, 24));
            sin_latitude = pow(2, -30)*static_cast<double>(get_bits<int32_t>(words, 227, 21));
            cos_latitude = pow(2, -30)*static_cast<double>(get_bits<int32_t>(words, 248, 21));
        }
        std::string to_string() const;

    private:
        uint16_t ephemeris_time_of_week;
        double longitude_orbit_plane;
        double rate_right_ascension_diff;
        double inclination_angle;
        double rate_inclination_angle;
        double sin_inclination;
        double cos_inclination;
        double sin_orbit_radius;
        double cos_orbit_radius;
        double sin_latitude;
        double cos_latitude;
};

class message_10
{
    private:

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
};

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
        double rate_inclination_angle;
        double sin_inclination;
        double cos_inclination;
        double sin_orbit_radius;
        double cos_orbit_radius;
        double sin_latitude;
        double cos_latitude;

    public:
        void update_location() override;
        std::string to_string() const override;
};

class L1CAEphemeris: public EphemerisInterface
{
    private:
        // Subframe 2
        double sin_orbit_radius;
        double mean_motion_diff;
        double mean_anomaly;
        double cos_latitude;
        double eccentricity;
        double sin_latitude;
        double sqrt_semi_major_axis;
        uint32_t ephemeris_time_of_week;

        // Subframe 2,3
        uint8_t IODE;

        // Subframe 3
        double cos_inclination;
        double longitude_orbit_plane;
        double sin_inclination;
        double inclination_angle;
        double cos_orbit_radius;
        double argument_of_perigee;
        double rate_right_ascension;
        double rate_inclination_angle;

    public:
        void update_location() override;
        std::string to_string() const override; 
};

}

}

#endif