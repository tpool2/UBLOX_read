#include "GNSS/gnss.hpp"
#include <iostream>

namespace gnss
{
    using bit_utils::get_msb_bits;
    
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
        if(get_msb_bits<uint8_t>(words, 0, 8) == gps::kPreamble)
        {
            parse_L2(words);
        }
        else if(get_msb_bits<uint8_t>(words, 2, 8) == gps::kPreamble)
        {
            // gps::parse_l1_ca(words);
        }
        else
        {
            std::cout<<"Unknown"<<std::endl;
        }
    }

    void gps::parse_l1_ca(const uint32_t* words)
    {
        // std::cout<<"L1 C/A"<<std::endl;
        throw std::logic_error("Not implemented correctly yet");
        int tow_truncated = get_msb_bits<int>(words, 2, 19);
        // std::cout<<"TOW-Count: "<<tow_truncated<<std::endl;
        int subframe_id = get_msb_bits<int>(words, 21, 24);
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
        return static_cast<int32_t>((get_msb_bits<uint32_t>(words, position, 8)<<24) | get_msb_bits<uint32_t>(words, position+14, 24));
    }

    void gps::parse_subframe_3(const uint32_t* words)
    {
        int16_t C_ic = get_msb_bits<int16_t>(words, 60, 16);
        // std::cout << "C_ic: " << C_ic << std::endl;
        int32_t Omega_0 = l1_get_split_data(words, 76);
        // std::cout << "Omega_0: " << Omega_0 << std::endl;
        int16_t C_is = get_msb_bits<int16_t>(words, 120, 16);
        // std::cout << "C_is: " << C_is << std::endl;
        int32_t i_0 = l1_get_split_data(words, 136);
        // std::cout << "i_0: " << i_0 << std::endl;
        int16_t C_rc = get_msb_bits<int16_t>(words, 180, 16);
        // std::cout << "C_rc: " << C_rc<<std::endl;
        int32_t omega = l1_get_split_data(words, 196);
        // std::cout << "omega: " << omega << std::endl;
        int32_t Omega_dot = get_msb_bits<int32_t>(words, 240, 24);
        // std::cout << "Omega_dot: " << Omega_dot << std::endl;
        uint8_t IODE = get_msb_bits<uint8_t>(words, 270, 8);
        // std::cout << "IODE: " << static_cast<uint16_t>(IODE) << std::endl;
        int16_t IDOT = get_msb_bits<int16_t>(words, 278, 14);
        // std::cout<<"IDOT: " << IDOT << std::endl;
    }

    void NavParser::parse_L2(const uint32_t* words)
    {
        int prn = get_msb_bits<int>(words, 8, 6);
        int message_type_id = get_msb_bits<int>(words, 14, 6);
        int message_tow_count = get_msb_bits<int>(words, 20, 17);
        switch (message_type_id)
        {
            case 10:
            {
                auto message = std::make_shared<gps::message_10>(gps::parse_L2_10(words));
                message_10_map.insert_or_assign(message->get_prn(), message);
            } break;
            case 11:
            {
                auto my_message_11 = std::make_shared<gps::message_11>(gps::parse_L2_11(words));
                if(message_10_map.count(my_message_11->get_prn()) > 0)
                {
                    auto my_message_10 = message_10_map[my_message_11->get_prn()];
                    gps::L2Ephemeris eph(my_message_10, my_message_11);
                    std::cout<<eph.to_string()<<std::endl;
                }
            } break;
            default:
            {

            } break;
        }
    }

    gps::message_10 gps::parse_L2_10(const uint32_t* words)
    {
        message_10 my_message(words);
        return my_message;
    }

    gps::message_11 gps::parse_L2_11(const uint32_t* words)
    {
        message_11 my_message(words);
        return my_message;
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