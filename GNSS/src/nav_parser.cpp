#include "GNSS/gnss.hpp"
#include <iostream>

namespace gnss
{
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
        std::cout<<"Reading GPS Message with "<< num_words << " words" <<std::endl;
        if((words[0] & (0xFF)<<24)>>24 == gps::kPreamble)
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
        std::cout<<"L1 C/A"<<std::endl;
        int tow_truncated = bit_utils::get_bits_msb(words[1], 2, 19);
        std::cout<<"TOW-Count: "<<tow_truncated<<std::endl;
        int subframe_id = bit_utils::get_bits_msb(words[1], 21, 24);
        std::cout<<"Subframe ID: "<<subframe_id<<std::endl;
        switch(subframe_id)
        {
            case 3:
                parse_subframe_3(words);
                break;
            default:
                break;
        }
    }

    void gps::parse_subframe_3(const uint32_t* words)
    {
        int16_t C_ic = bit_utils::get_bits_msb(words[2], 2, 18);
        std::cout<<"C_ic: "<<C_ic << std::endl;
        int32_t Omega_0 = (bit_utils::get_bits_msb(words[2], 18, 26)<<24) | bit_utils::get_bits_msb(words[3], 2, 26);
        std::cout<<"Omega_0: "<<Omega_0 << std::endl;
        int16_t C_is = bit_utils::get_bits_msb(words[4], 2, 18);
        std::cout<<"C_is: "<<C_is << std::endl;
        int32_t i_0 = (bit_utils::get_bits_msb(words[4], 18, 26)<<24) | bit_utils::get_bits_msb(words[5], 2, 26);
        std::cout<<"i_0: "<<i_0<<std::endl;
    }

    void gps::parse_l2c(const uint32_t* words)
    {
        std::cout<<"L2"<<std::endl;
        int prn = bit_utils::get_bits_msb(words[0], 8, 14);
        std::cout<<"PRN: " << prn << std::endl;
        int message_type_id = bit_utils::get_bits_msb(words[0], 14, 20);
        std::cout<<"Message Type ID: " << message_type_id << std::endl;
        int message_tow_count = ((uint32_t(bit_utils::get_bits_msb(words[0], 20, 32))<<5) | bit_utils::get_bits_msb(words[1], 0, 5) );
        std::cout<<"Message TOW Count: " << message_tow_count << std::endl;
        std::cout<<"SV Time: "<< message_tow_count*6 << std::endl;
        std::cout<<"Alert Flag: " << bit_utils::get_bits_msb(words[1], 5, 6) << std::endl;
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
        int t_oe = bit_utils::get_bits_msb(words[1], 6, 17);
        std::cout<<"t_oe: "<<t_oe<<std::endl;
        int Omega_0_n = (bit_utils::get_bits_msb(words[1], 17, 32)<<15 | bit_utils::get_bits_msb(words[2], 0, 18));
        std::cout<<"Omega_0_n: "<<Omega_0_n<<std::endl;
    }

    void NavParser::parse_handover_word(const std::bitset<30> &word)
    {

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