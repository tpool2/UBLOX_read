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
    double semi_major_axis = A_ref + semi_major_axis_diff;
    double mean_motion = sqrt(mu/std::pow(semi_major_axis, 3));
}

void L2C_Message::parse_preamble(const uint32_t* words)
{
    prn = get_msb_bits<uint8_t>(words, 8, 6);
    message_type_id = get_msb_bits<uint8_t>(words, 14, 6);
    message_tow = get_msb_bits<uint32_t>(words, 20, 17);
    alert_flag = get_msb_bits<bool>(words, 37, 1);
}

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
    ss<<"ED Accuracy Index: " << ED_accuracy << std::endl;
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
}}