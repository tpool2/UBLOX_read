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

void CNAVEphemeris::update_location()
{
    double A_0 = A_ref + msg_10->get_semi_major_axis_diff();
    int t = msg_11->get_message_tow()*6-12;
    int t_k = t - msg_11->get_ephemeris_time_of_week();
    double A_k = A_0 + msg_10->get_semi_major_axis_change_rate()*t_k;
    double n_0 = sqrt(mu/pow(A_0, 3));
    
}

void CNAV_Message::parse_preamble(const uint32_t* words)
{
    prn = get_msb_bits<uint8_t>(words, 8, 6);
    message_type_id = get_msb_bits<uint8_t>(words, 14, 6);
    message_tow = get_msb_bits<uint32_t>(words, 20, 17);
    alert_flag = get_msb_bits<bool>(words, 37, 1);
}

uint8_t CNAV_Message::get_prn() const { return prn; }
uint8_t CNAV_Message::get_message_type_id() const { return message_type_id; }
uint32_t CNAV_Message::get_message_tow() const { return message_tow; }
bool CNAV_Message::get_alert_flag() const { return alert_flag; }
uint32_t CNAV_Message::get_ephemeris_time_of_week() const { return ephemeris_time_of_week; }

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
    ss << preamble_to_string();
    ss << "Ephemeris Time of Week: "<< ephemeris_time_of_week <<std::endl;
    ss << "Data Sequence Propogation Week Number: " << data_sequence_propogation_week_number << std::endl;
    ss << "L1 Health: " << l1_health << std::endl;
    ss << "L2 Health: " << l2_health << std::endl;
    ss << "L5 Health: " <<l5_health << std::endl;
    ss << "CEI Data Sequence Propogation Time of Week: " << CEI_time_of_week << std::endl;
    ss << "ED Accuracy Index: " << static_cast<int16_t>(ED_accuracy) << std::endl;
    ss << "Semi-major Axis Difference: " << semi_major_axis_diff << std::endl;
    ss << "Semi-major Axis Change Rate: " << semi_major_axis_change_rate << std::endl;
    ss << "Mean Motion Difference From Computed Value: " << mean_motion_diff << std::endl;
    ss << "Mean Motion Difference Rate: " << mean_motion_diff_rate << std::endl;
    ss << "Mean Anomaly: " << mean_anomaly << std::endl;
    ss << "Eccentricity: " << eccentricity << std::endl;
    ss << "Argument of Perigee: " << argument_of_perigee << std::endl;
    return ss.str();
}

std::string message_11::to_string() const
{
    std::stringstream ss;
    ss << preamble_to_string();
    ss << "Ephemeris Time of Week: "<< ephemeris_time_of_week <<std::endl;
    ss << "Longitude of Ascending Node of Orbit Plane at Weekly Epoch: "<< longitude_orbit_plane <<std::endl;
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

uint32_t message_30::get_CEI_time_of_week() const { return CEI_time_of_week;}
int8_t message_30::get_URA_NED_accuracy_index() const { return URA_NED_accuracy_index;}
uint8_t message_30::get_URA_NED_accuracy_change_index() const { return URA_NED_accuracy_change_index;}
uint8_t message_30::get_URA_NED_accuracy_change_rate_index() const { return URA_NED_accuracy_change_rate_index;}
uint32_t message_30::get_clock_data_ref_time_of_week() const { return clock_data_ref_time_of_week;}
double message_30::get_clock_bias_correction() const { return clock_bias_correction;}
double message_30::get_clock_drift_correction() const { return clock_drift_correction;}
double message_30::get_clock_drift_rate_correction() const { return clock_drift_rate_correction;}
double message_30::get_group_delay_differential() const { return group_delay_differential;}
double message_30::get_ISC_L1_CA() const { return ISC_L1_CA;}
double message_30::get_ISC_L2C() const { return ISC_L2C;}
double message_30::get_ISC_L5I5() const { return ISC_L5I5;}
double message_30::get_ISC_L5Q5() const { return ISC_L5Q5;}

std::string message_30::to_string() const
{
    std::stringstream ss;
    ss << preamble_to_string();
    ss << "CEI Data Sequence Propogation Time of Week: " << CEI_time_of_week << std::endl;
    ss << "URA NED Accuracy Index: " << static_cast<int16_t>(URA_NED_accuracy_index) << std::endl;
    ss << "URA NED Accuracy Change Index: " << static_cast<uint16_t>(URA_NED_accuracy_change_index) << std::endl;
    ss << "URA NED Accuracy Change Rate Index: " << static_cast<uint16_t>(URA_NED_accuracy_change_rate_index) << std::endl;
    ss << "Clock Data Reference Time of Week: " << clock_data_ref_time_of_week << std::endl;
    ss << "SV Clock Drift Correction Coefficient: " << clock_drift_correction << std::endl;
    ss << "SV Clock Drift Rate Correction Coefficient: " << clock_drift_rate_correction << std::endl;
    ss << "SV Clock Bias Correction Coefficient: " << clock_bias_correction << std::endl;
    ss << "Group Delay Differential: " << group_delay_differential << std::endl;
    
    return ss.str();
}

std::string CNAV_Message::preamble_to_string() const
{
    std::stringstream ss;
    ss<<"PRN: " << static_cast<uint16_t>(prn) << std::endl;
    ss<<"Message Type ID: " << static_cast<uint16_t>(message_type_id) << std::endl;
    ss<<"Message TOW: " << message_tow << std::endl;
    ss<<"Alert Flag: " << alert_flag << std::endl;
    return ss.str();
}

std::string CNAVEphemeris::to_string() const
{
    std::stringstream ss;
    ss<<msg_10->to_string()<<msg_11->to_string();
    return ss.str();

}
}}