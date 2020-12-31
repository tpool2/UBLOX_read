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
        for(int word_index = 0; word_index < num_words; ++word_index)
        {
            // std::cout<<words[word_index]<<"\t";
            std::bitset<30> word = words[word_index];
            word = bit_utils::flip_endian(word);
            for(int bit_index = 0; bit_index < 30; ++bit_index)
            {
                std::cout<<word[bit_index]<<" ";
            }
            std::cout<<"\t";
        }
        std::cout<<std::endl;
    }

    /*
    Checks GPS Parity given a 30-bit word, and the previous word's 29th and 30th bits in the subframe
    */
    bool check_parity(const std::bitset<30> &bits, const bool &prev_29, const bool &prev_30)
    {
        std::bitset<30> b;
        for(int index = 0; index < sizeof(bits); ++index)
        {   
            b[index] = bits[index]^prev_30;
        }
        return (b[24] == (prev_29^b[0]^b[1]^b[2]^b[5]^b[6]^b[9]^b[10]^b[11]^b[12]^b[13]^b[16]^b[17]^b[19]^b[22]))
            & (b[25] == (prev_30^b[1]^b[2]^b[3]^b[5]^b[6]^b[10]^b[11]^b[12]^b[13]^b[14]^b[17]^b[18]^b[20]^b[24]))
            & (b[26] == (prev_29^b[0]^b[2]^b[3]^b[4]^b[6]^b[7]^b[11]^b[12]^b[13]^b[14]^b[15]^b[18]^b[19]^b[21]))
            & (b[27] == (prev_30^b[1]^b[3]^b[4]^b[5]^b[7]^b[8]^b[12]^b[13]^b[14]^b[15]^b[16]^b[19]^b[20]^b[22]))
            & (b[28] == (prev_30^b[0]^b[2]^b[4]^b[5]^b[6]^b[8]^b[9]^b[13]^b[14]^b[15]^b[16]^b[17]^b[20]^b[21]^b[23]))
            & (b[29] == (prev_29^b[2]^b[4]^b[5]^b[7]^b[8]^b[9]^b[10]^b[12]^b[14]^b[18]^b[21]^b[22]^b[23]));
    }
}