#ifndef EXTENSIVE_CODING_H
#define EXTENSIVE_CODING_H

#include <stdint.h>

namespace corecvs {



/**
 *   so far this is a very limited use class.
 *
 *   It encodes 7 bit of data to 10
 *
 **/
class ExtensiveCoding
{
public:
    ExtensiveCoding();

    static uint32_t encode (uint32_t input);
    static uint32_t decode (uint32_t input);

};

} // namespace corecvs

#endif // EXTENSIVE_CODING_H
