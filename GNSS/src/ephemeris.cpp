#include "GNSS/gnss.hpp"
#include <iostream>

namespace gnss{ 

double EphemerisInterface::get_x_ecef() const
{
    return x_ecef;
}

double EphemerisInterface::get_y_ecef() const
{
    return y_ecef;
}

double EphemerisInterface::get_z_ecef() const
{
    return z_ecef;
}

namespace gps
{

void L2Ephemeris::update_location()
{

}

void L2C_Message::parse_preamble(const uint32_t* words)
{
    prn = get_msb_bits<uint8_t>(words, 8, 6);
    message_type_id = get_msb_bits<uint8_t>(words, 14, 6);
    message_tow = get_msb_bits<uint32_t>(words, 20, 17);
    alert_flag = get_msb_bits<bool>(words, 37, 1);
}

uint8_t L2C_Message::get_prn() const { return prn; }
uint8_t L2C_Message::get_message_type_id() const { return message_type_id; }
uint32_t L2C_Message::get_message_tow() const { return message_tow; }
bool L2C_Message::get_alert_flag() const { return alert_flag; }
uint32_t L2C_Message::get_ephemeris_time_of_week() const { return ephemeris_time_of_week; }

uint16_t message_10::get_data_sequence_propogation_week_number() const { return data_sequence_propogation_week_number; }
bool message_10::get_l1_health() const { return l1_health; }
bool message_10::get_l2_health() const { return l2_health; }
bool message_10::get_l5_health() const { return l5_health; }
int8_t message_10::get_ED_accuracy() const { return ED_accuracy; }
uint32_t message_10::get_CEI_time_of_week() const { return CEI_time_of_week; }
double message_10::get_semi_major_axis_diff() const { return semi_major_axis_diff; }
double message_10::get_semi_major_axis_change_rate() const { return semi_major_axis_change_rate; }
double message_10::get_mean_motion_diff() const { return mean_motion_diff; }
double message_10::get_mean_motion_diff_rate() const { return mean_motion_diff_rate; }
double message_10::get_mean_anomaly() const { return mean_anomaly; }
double message_10::get_eccentricity() const { return eccentricity; }
double message_10::get_argument_of_perigee() const { return argument_of_perigee; }

double message_11::get_longitude_orbit_plane() const { return longitude_orbit_plane; }
double message_11::get_rate_right_ascension_diff() const { return rate_right_ascension_diff; }
double message_11::get_inclination_angle() const { return inclination_angle; }
double message_11::get_rate_inclination_angle() const { return rate_inclination_angle; }
double message_11::get_sin_inclination() const { return sin_inclination; }
double message_11::get_cos_inclination() const { return cos_inclination; }
double message_11::get_sin_orbit_radius() const { return sin_orbit_radius; }
double message_11::get_cos_orbit_radius() const { return cos_orbit_radius; }
double message_11::get_sin_latitude() const { return sin_latitude; }
double message_11::get_cos_latitude() const { return cos_latitude; }

std::string message_10::to_string() const
{
    std::stringstream ss;
    ss<<"PRN: " << static_cast<uint16_t>(prn) << std::endl;
    ss<<"Message Type ID: " << static_cast<uint16_t>(message_type_id) << std::endl;
    ss<<"Message TOW: " << message_tow << std::endl;
    ss<<"Alert Flag: " << alert_flag << std::endl;
    ss<<"Ephemeris Time of Week: "<< ephemeris_time_of_week <<std::endl;
    ss<<"Data Sequence Propogation Week Number: " << data_sequence_propogation_week_number << std::endl;
    ss<<"L1 Health: " << l1_health << std::endl;
    ss<<"L2 Health: " << l2_health << std::endl;
    ss<<"L5 Health: " <<l5_health << std::endl;
    ss<<"CEI Data Sequence Propogation Time of Week: " << CEI_time_of_week << std::endl;
    ss<<"ED Accuracy Index: " << static_cast<int16_t>(ED_accuracy) << std::endl;
    ss<<"Semi-major Axis Difference: " << semi_major_axis_diff << std::endl;
    ss<<"Semi-major Axis Change Rate: " << semi_major_axis_change_rate << std::endl;
    ss<<"Mean Motion Difference From Computed Value: " << mean_motion_diff << std::endl;
    ss<<"Mean Motion Difference Rate: " << mean_motion_diff_rate << std::endl;
    ss<<"Mean Anomaly: " << mean_anomaly << std::endl;
    ss<<"Eccentricity: " << eccentricity << std::endl;
    ss<<"Argument of Perigee: " << argument_of_perigee << std::endl;
    return ss.str();
}

std::string message_11::to_string() const
{
    std::stringstream ss;
    ss<<"PRN: " << static_cast<uint16_t>(prn) << std::endl;
    ss<<"Message Type ID: " << static_cast<uint16_t>(message_type_id) << std::endl;
    ss<<"Message TOW: " << message_tow << std::endl;
    ss<<"Alert Flag: " << alert_flag << std::endl;
    ss<<"Ephemeris Time of Week: "<< ephemeris_time_of_week <<std::endl;
    ss<<"Longitude of Ascending Node of Orbit Plane at Weekly Epoch: "<< longitude_orbit_plane <<std::endl;
    ss << "Inclination Angle at Reference Time: " << inclination_angle << std::endl;
    ss << "Rate of Right Ascension Difference: " << rate_right_ascension_diff << std::endl;
    ss << "Rate of Inclination Angle: " << rate_inclination_angle << std::endl;
    ss << "Sine Inclination: " << sin_inclination << std::endl;
    ss << "Cosine Inclination: " << cos_inclination << std::endl;
    ss << "Sine Orbit Radius: " << sin_orbit_radius << std::endl;
    ss << "Cosine Orbit Radius: " << cos_orbit_radius << std::endl;
    ss << "Sine Latitude: " << sin_latitude << std::endl;
    ss << "Cosine Latitude: " << cos_latitude << std::endl;
    return ss.str();
}

std::string L2Ephemeris::to_string() const
{
    std::stringstream ss;
    ss<<msg_10->to_string()<<msg_11->to_string();
    return ss.str();

}
}}