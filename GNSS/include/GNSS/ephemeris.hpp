#ifndef EPHEMERIS_HPP
#define EPHEMERIS_HPP
#include <cstdint>
#include <string>
#include <sstream>
#include <memory>
#include "bit_utils/bit_utils.h"

using bit_utils::get_msb_bits;

namespace gnss
{

class EphemerisInterface
{
    protected:
        double x_ecef;
        double y_ecef;
        double z_ecef;
        uint8_t prn;
    
    public:
        virtual void update_location() = 0;
        double get_x_ecef() const;
        double get_y_ecef() const;
        double get_z_ecef() const;
        virtual std::string to_string() const = 0;
};

namespace gps
{
class CNAV_Message
{
    public:
        virtual std::string to_string() const = 0;
        uint8_t get_prn() const;
        uint8_t get_message_type_id() const;
        uint32_t get_message_tow() const;
        bool get_alert_flag() const;
        uint32_t get_ephemeris_time_of_week() const;
        std::string preamble_to_string() const;
        
    protected:
        uint8_t prn;
        uint8_t message_type_id;
        uint32_t message_tow;
        bool alert_flag;
        uint32_t ephemeris_time_of_week;
        void parse_preamble(const uint32_t* words);
};

class message_30: public CNAV_Message
{
    public:
        message_30(const uint32_t* words)
        {
            parse_preamble(words);
            CEI_time_of_week = 300*static_cast<uint32_t>(get_msb_bits<uint16_t>(words, 38, 11));
            URA_NED_accuracy_index = get_msb_bits<int8_t>(words, 49, 5);
            URA_NED_accuracy_change_index = get_msb_bits<uint8_t>(words, 54, 3);
            URA_NED_accuracy_change_rate_index = get_msb_bits<uint8_t>(words, 57, 3);
            clock_data_ref_time_of_week = 300*static_cast<uint32_t>(get_msb_bits<uint16_t>(words, 60, 11));
            clock_bias_correction = pow(2, -35)*static_cast<double>(get_msb_bits<int32_t>(words, 71, 26));
            clock_drift_correction = pow(2, -48)*static_cast<double>(get_msb_bits<int32_t>(words, 97, 20));
            clock_drift_rate_correction = pow(2, -60)*static_cast<double>(get_msb_bits<int16_t>(words, 117, 10));
            group_delay_differential = pow(2, -35)*static_cast<double>(get_msb_bits<int16_t>(words, 127, 13));
            ISC_L1_CA = pow(2, -35)*static_cast<double>(get_msb_bits<int16_t>(words, 140, 13));
            ISC_L2C = pow(2, -35)*static_cast<double>(get_msb_bits<int16_t>(words, 153, 13));
            ISC_L5I5 = pow(2, -35)*static_cast<double>(get_msb_bits<int16_t>(words, 166, 13));
            ISC_L5Q5 = pow(2, -35)*static_cast<double>(get_msb_bits<int16_t>(words, 179, 13));
        }

        uint32_t get_CEI_time_of_week() const;
        int8_t get_URA_NED_accuracy_index() const;
        uint8_t get_URA_NED_accuracy_change_index() const;
        uint8_t get_URA_NED_accuracy_change_rate_index() const;
        uint32_t get_clock_data_ref_time_of_week() const;
        double get_clock_bias_correction() const;
        double get_clock_drift_correction() const;
        double get_clock_drift_rate_correction() const;
        double get_group_delay_differential() const;
        double get_ISC_L1_CA() const;
        double get_ISC_L2C() const;
        double get_ISC_L5I5() const;
        double get_ISC_L5Q5() const;

        

        std::string to_string() const override;


    private:
        uint32_t CEI_time_of_week;
        int8_t URA_NED_accuracy_index;
        uint8_t URA_NED_accuracy_change_index;
        uint8_t URA_NED_accuracy_change_rate_index;
        uint32_t clock_data_ref_time_of_week;
        double clock_bias_correction;
        double clock_drift_correction;
        double clock_drift_rate_correction;
        double group_delay_differential;
        double ISC_L1_CA;
        double ISC_L2C;
        double ISC_L5I5;
        double ISC_L5Q5;
};

class message_11: public CNAV_Message
{
    public:
        message_11(const uint32_t* words)
        {
            parse_preamble(words);
            ephemeris_time_of_week = 300*static_cast<uint32_t>(get_msb_bits<uint16_t>(words, 38, 11));
            longitude_orbit_plane = pow(2,-32)*static_cast<double>(get_msb_bits<int64_t>(words, 49, 33));
            inclination_angle = pow(2, -32)*static_cast<double>(get_msb_bits<int64_t>(words, 82, 33));
            rate_right_ascension_diff = pow(2, -44)*static_cast<double>(get_msb_bits<int32_t>(words, 115, 17));
            rate_inclination_angle = pow(2, -44)*static_cast<double>(get_msb_bits<int16_t>(words, 15));
            sin_inclination = pow(2, -30)*static_cast<double>(get_msb_bits<int16_t>(words, 147, 16));
            cos_inclination = pow(2, -30)*static_cast<double>(get_msb_bits<int16_t>(words, 163, 16));
            sin_orbit_radius = pow(2, -8)*static_cast<double>(get_msb_bits<int32_t>(words, 179, 24));
            cos_orbit_radius = pow(2, -8)*static_cast<double>(get_msb_bits<int32_t>(words, 203, 24));
            sin_latitude = pow(2, -30)*static_cast<double>(get_msb_bits<int32_t>(words, 227, 21));
            cos_latitude = pow(2, -30)*static_cast<double>(get_msb_bits<int32_t>(words, 248, 21));
        }
        double get_longitude_orbit_plane() const;
        double get_rate_right_ascension_diff() const;
        double get_inclination_angle() const;
        double get_rate_inclination_angle() const;
        double get_sin_inclination() const;
        double get_cos_inclination() const;
        double get_sin_orbit_radius() const;
        double get_cos_orbit_radius() const;
        double get_sin_latitude() const;
        double get_cos_latitude() const;
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

class message_10: public CNAV_Message
{
    public:
        message_10(const uint32_t* words)
        {
            parse_preamble(words);
            data_sequence_propogation_week_number = get_msb_bits<uint16_t>(words, 38, 13);
            l1_health = get_msb_bits<bool>(words, 51, 1);
            l2_health = get_msb_bits<bool>(words, 52, 1);
            l5_health = get_msb_bits<bool>(words, 53, 1);
            CEI_time_of_week = 300*static_cast<uint32_t>(get_msb_bits<uint16_t>(words, 54, 11));
            ED_accuracy = get_msb_bits<int8_t>(words, 65, 5);
            ephemeris_time_of_week = 300*static_cast<uint32_t>(get_msb_bits<uint16_t>(words, 70, 11));
            semi_major_axis_diff = pow(2, -9)*static_cast<double>(get_msb_bits<int32_t>(words, 81, 26));
            semi_major_axis_change_rate = pow(2, -21)*static_cast<double>(get_msb_bits<int32_t>(words, 107, 25));
            mean_motion_diff = pow(2, -44)*static_cast<double>(get_msb_bits<int32_t>(words, 132, 17));
            mean_motion_diff_rate = pow(2, -57)*static_cast<double>(get_msb_bits<int32_t>(words, 149, 23));
            mean_anomaly = pow(2, -32)*static_cast<double>(get_msb_bits<int64_t>(words, 172, 33));
            eccentricity = pow(2, -34)*static_cast<double>(get_msb_bits<uint64_t>(words, 205, 33));
            argument_of_perigee = pow(2, -32)*static_cast<double>(get_msb_bits<int64_t>(words, 238, 33));
        }
        uint16_t get_data_sequence_propogation_week_number() const;
        bool get_l1_health() const;
        bool get_l2_health() const;
        bool get_l5_health() const;
        int8_t get_ED_accuracy() const;
        uint32_t get_CEI_time_of_week() const;
        double get_semi_major_axis_diff() const;
        double get_semi_major_axis_change_rate() const;
        double get_mean_motion_diff() const;
        double get_mean_motion_diff_rate() const;
        double get_mean_anomaly() const;
        double get_eccentricity() const;
        double get_argument_of_perigee() const;
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

class CNAVEphemeris: public EphemerisInterface
{
    private:
        // Message 10
        std::shared_ptr<gps::message_11> msg_11;

        // Message 11
        std::shared_ptr<gps::message_10> msg_10;

        std::shared_ptr<gps::message_30> msg_30;

    public:
        CNAVEphemeris()
        {
            
        }
        CNAVEphemeris(std::shared_ptr<gps::message_10> msg_10, std::shared_ptr<gps::message_11> msg_11)
        {
            this->msg_10 = msg_10;
            this->msg_11 = msg_11;

            std::cout<<"Created L2 Ephemeris!"<<std::endl;

            update_location();
        }
        void update_message_30(std::shared_ptr<gps::message_30> msg_30);
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