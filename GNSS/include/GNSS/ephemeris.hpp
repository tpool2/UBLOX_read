#ifndef EPHEMERIS_HPP
#define EPHEMERIS_HPP
#include <cstdint>
#include <string>
#include <sstream>
#include <memory>
#include <iostream>
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

}

#endif