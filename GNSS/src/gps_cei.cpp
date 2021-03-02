#include "GNSS/gps_cei.hpp"

namespace gnss { namespace gps { namespace lnav {

bool update(CEI& cei, uint32_t* words)
{
    std::cout<<get_bits<uint16_t>(words, 48, 3)<<std::endl;
    return true;
}

} // namespace lnav
} // namespace gps
} // namespace gnss