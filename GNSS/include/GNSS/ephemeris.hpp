#ifndef EPHEMERIS_HPP
#define EPHEMERIS_HPP
#include <cstdint>
#include <string>
#include <sstream>
#include "bit_utils/bit_utils.h"

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
    public:
        virtual std::string to_string() const = 0; 
    protected:
        uint8_t prn;
        uint8_t message_type_id;
        uint32_t message_tow;
        bool alert_flag;
        uint32_t ephemeris_time_of_week;
        void parse_preamble(const uint32_t* words);
};

class message_11: public L2C_Message
{
    public:
        message_11(const uint32_t* words)
        {
            parse_preamble(words);
            ephemeris_time_of_week = 300*static_cast<uint32_t>(get_bits<uint16_t>(words, 38, 11));
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
        std::string to_string() const override;

    private:
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

class message_10: public L2C_Message
{
    public:
        message_10(const uint32_t* words)
        {
            parse_preamble(words);
            data_sequence_propogation_week_number = get_bits<uint16_t>(words, 38, 13);
            l1_health = get_bits<bool>(words, 51, 1);
            l2_health = get_bits<bool>(words, 52, 1);
            l5_health = get_bits<bool>(words, 53, 1);
            CEI_time_of_week = 300*static_cast<uint32_t>(get_bits<uint16_t>(words, 54, 11));
            ED_accuracy = get_bits<int8_t>(words, 65, 5);
            ephemeris_time_of_week = 300*static_cast<uint32_t>(get_bits<uint16_t>(words, 70, 11));
            semi_major_axis_diff = pow(2, -9)*static_cast<double>(get_bits<int32_t>(words, 81, 26));
            auto semi_major_axis_change_rate = pow(2, -21)*static_cast<double>(get_bits<int32_t>(words, 107, 25));
            auto mean_motion_diff = pow(2, -44)*static_cast<double>(get_bits<int32_t>(words, 132, 17));
            auto mean_motion_diff_rate = pow(2, -57)*static_cast<double>(get_bits<int32_t>(words, 149, 23));
            auto mean_anomaly = pow(2, -32)*static_cast<double>(get_bits<int64_t>(words, 172, 33));
            auto eccentricity = pow(2, -34)*static_cast<double>(get_bits<uint64_t>(words, 205, 33));
            auto argument_of_perigee = pow(2, -32)*static_cast<double>(get_bits<int64_t>(words, 238, 33));
        }

        std::string to_string() const override;
    
    private:
        uint16_t data_sequence_propogation_week_number;
        bool l1_health;
        bool l2_health;
        bool l5_health;
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