#include "Bit_Utils/bit_utils.h"

namespace bit_utils
{
    bool little_endian()
    {
        int i = 1;
        char *p = (char *)&i;
        return p[0]==1;
    }
}