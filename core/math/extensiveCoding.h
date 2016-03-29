#ifndef EXTENSIVE_CODING_H
#define EXTENSIVE_CODING_H

#include <stdlib.h>
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
    ExtensiveCoding(int _inbits = 7);

    /**
     *   Lut for every 10bits holds the 7bit of decoded data
     **/
    uint32_t *LUT;
    uint lutsize;
    uint inbits;
    uint addbits;
    uint outbits;

    static uint32_t encode (uint32_t input);
    static uint32_t decode (uint32_t input);


    ~ExtensiveCoding();
};

} // namespace corecvs

#endif // EXTENSIVE_CODING_H
