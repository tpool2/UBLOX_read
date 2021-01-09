#include "GNSS/gnss.hpp"
#include <iostream>

namespace gnss
{
    using bit_utils::get_bits;
    
    void NavParser::parse_sfrbx(const ublox::ubx::UBX_message_t& message)
    {
        uint8_t gnss_id = message.payload.RXM_SFRBX.gnssId;
        if(gnss_id == ublox::ubx::kGnssID_GPS)
        {
            read_gps_message(message.payload.RXM_SFRBX.dwrd, message.payload.RXM_SFRBX.numWords);
        }
        else
        {
            
        }
    }

    void NavParser::read_gps_message(const uint32_t* words, size_t num_words)
    {
        // std::cout<<"Reading GPS Message with "<< num_words << " words" <<std::endl;
        if(get_bits<uint8_t>(words, 0, 8) == gps::kPreamble)
        {
            gps::parse_l2c(words);
        }
        else if((words[0] & (0xFF)<<22)>>22 == gps::kPreamble)
        {
            gps::parse_l1_ca(words);
        }
        else
        {
            std::cout<<"Unknown"<<std::endl;
        }
    }

    void gps::parse_l1_ca(const uint32_t* words)
    {
        // std::cout<<"L1 C/A"<<std::endl;
        int tow_truncated = bit_utils::get_bits_msb(words[1], 2, 19);
        // std::cout<<"TOW-Count: "<<tow_truncated<<std::endl;
        int subframe_id = bit_utils::get_bits_msb(words[1], 21, 24);
        // std::cout<<"Subframe ID: "<<subframe_id<<std::endl;
        switch(subframe_id)
        {
            case 3:
                parse_subframe_3(words);
                break;
            default:
                break;
        }
    }

    int32_t gps::l1_get_split_data(const uint32_t* words, int position)
    {
        return static_cast<int32_t>((get_bits<uint32_t>(words, position, 8)<<24) | get_bits<uint32_t>(words, position+14, 24));
    }

    void gps::parse_subframe_3(const uint32_t* words)
    {
        int16_t C_ic = get_bits<int16_t>(words, 60, 16);
        // std::cout << "C_ic: " << C_ic << std::endl;
        int32_t Omega_0 = l1_get_split_data(words, 76);
        // std::cout << "Omega_0: " << Omega_0 << std::endl;
        int16_t C_is = get_bits<int16_t>(words, 120, 16);
        // std::cout << "C_is: " << C_is << std::endl;
        int32_t i_0 = l1_get_split_data(words, 136);
        // std::cout << "i_0: " << i_0 << std::endl;
        int16_t C_rc = get_bits<int16_t>(words, 180, 16);
        // std::cout << "C_rc: " << C_rc<<std::endl;
        int32_t omega = l1_get_split_data(words, 196);
        // std::cout << "omega: " << omega << std::endl;
        int32_t Omega_dot = get_bits<int32_t>(words, 240, 24);
        // std::cout << "Omega_dot: " << Omega_dot << std::endl;
        uint8_t IODE = get_bits<uint8_t>(words, 270, 8);
        // std::cout << "IODE: " << static_cast<uint16_t>(IODE) << std::endl;
        int16_t IDOT = get_bits<int16_t>(words, 278, 14);
        // std::cout<<"IDOT: " << IDOT << std::endl;
    }

    void gps::parse_l2c(const uint32_t* words)
    {
        std::cout<<"L2"<<std::endl;
        int prn = get_bits<int>(words, 8, 6);
        std::cout<<"PRN: " << prn << std::endl;
        int message_type_id = get_bits<int>(words, 14, 6);
        // std::cout<<"Message Type ID: " << message_type_id << std::endl;
        int message_tow_count = get_bits<int>(words, 20, 17);
        // std::cout<<"Message TOW Count: " << message_tow_count << std::endl;
        // std::cout<<"SV Time: "<< message_tow_count*6 << std::endl;
        // std::cout<<"Alert Flag: " << bit_utils::get_bits_msb(words[1], 5, 6) << std::endl;
        switch (message_type_id)
        {
        case 11:
            parse_l2_nav_11(words);
            break;
        
        default:
            break;
        }
    }

    void gps::parse_l2_nav_11(const uint32_t* words)
    {
        auto t_oe = get_bits<uint16_t>(words, 38, 11);
        auto Omega_0_n = get_bits<int64_t>(words, 49, 33);
        auto inclination_angle = get_bits<int64_t>(words, 82, 33);
        auto rate_right_ascension_diff = get_bits<int32_t>(words, 115, 17);
        auto rate_inclination_angle = get_bits<int16_t>(words, 15);
        auto sin_inclination = get_bits<int16_t>(words, 147, 16);
        auto cos_inclination = get_bits<int16_t>(words, 163, 16);
        auto sin_orbit_radius = get_bits<int32_t>(words, 179, 24);
        auto cos_orbit_radius = get_bits<int32_t>(words, 203, 24);
        auto sin_latitude = get_bits<int32_t>(words, 227, 21);
        auto cos_latitude = get_bits<int32_t>(words, 248, 21);

        std::cout<<"t_oe: "<<t_oe<<std::endl;
        std::cout<<"Omega_0_n: "<<Omega_0_n<<std::endl;
        std::cout << "Inclination Angle at Reference Time: " << inclination_angle << std::endl;
        std::cout << "Rate of Right Ascension Difference: " << rate_right_ascension_diff << std::endl;
        std::cout << "Rate of Inclination Angle: " << rate_inclination_angle << std::endl;
        std::cout << "Sine Inclination: " << sin_inclination << std::endl;
        std::cout << "Cosine Inclination: " << cos_inclination << std::endl;
        std::cout << "Sine Orbit Radius: " << sin_orbit_radius << std::endl;
        std::cout << "Cosine Orbit Radius: " << cos_orbit_radius << std::endl;
        std::cout << "Sine Latitude: " << sin_latitude << std::endl;
        std::cout << "Cosine Latitude: " << cos_latitude << std::endl;
    }

    /*
    Checks GPS Parity given a 30-bit word, and the previous word's 29th and 30th bits in the subframe
    */
    bool check_parity(const std::bitset<30> &bits, const bool &prev_29, const bool &prev_30)
    {
        std::bitset<30> b = bits^std::bitset<30>(prev_30);
        b[24] = (prev_29^b[0]^b[1]^b[2]^b[4]^b[5]^b[9]^b[10]^b[11]^b[12]^b[13]^b[16]^b[17]^b[19]^b[22]);
        b[25] = (prev_30^b[1]^b[2]^b[3]^b[5]^b[6]^b[10]^b[11]^b[12]^b[13]^b[14]^b[17]^b[18]^b[20]^b[23]);
        b[26] = (prev_29^b[0]^b[2]^b[3]^b[4]^b[6]^b[7]^b[11]^b[12]^b[13]^b[14]^b[15]^b[18]^b[19]^b[21]);
        b[27] = (prev_30^b[1]^b[3]^b[4]^b[5]^b[7]^b[8]^b[12]^b[13]^b[14]^b[15]^b[16]^b[19]^b[20]^b[22]);
        b[28] = (prev_30^b[0]^b[2]^b[4]^b[5]^b[6]^b[8]^b[9]^b[13]^b[14]^b[15]^b[16]^b[17]^b[20]^b[21]^b[23]);
        b[29] = (prev_29^b[2]^b[4]^b[5]^b[7]^b[8]^b[9]^b[10]^b[12]^b[14]^b[18]^b[21]^b[22]^b[23]);
        for(int i = 0; i < 30; ++i)
        {
            std::cout<<bits[i];
        }
        std::cout<<" "<<bits.to_string()<<"\t"<<b.to_string()<<std::endl;
        return (bits[24] == b[24])
            & (bits[25] == b[25])
            & (bits[26] == b[26])
            & (bits[27] == b[27])
            & (bits[28] == b[28])
            & (bits[29] == b[29]);
    }
}