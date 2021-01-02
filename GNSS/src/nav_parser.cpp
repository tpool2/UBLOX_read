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
        bool d_29 = false;
        bool d_30 = false;
        for(int word_index = 0; word_index < num_words; ++word_index)
        {
            std::bitset<30> word = words[word_index];
            word = bit_utils::flip_endian(word);
            std::cout<<check_parity(word, d_29, d_30)<<"\n";
            d_29 = word[28];
            d_30 = word[29];
        }
        std::cout<<std::endl;
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