#ifndef GPS_LNAV_HPP
#define GPS_LNAV_HPP
#include "GNSS/gps_defs.h"
#include "bit_utils/bit_utils.h"
namespace gnss { namespace gps { namespace lnav {

bool check_parity(uint32_t word, bool D_29, bool D_30);

} // namespace lnav
} // namespace gps
} // namespace gnss

#endif