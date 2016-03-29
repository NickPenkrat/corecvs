#include "extensiveCoding.h"
#include "mathUtils.h"

namespace corecvs {

ExtensiveCoding::ExtensiveCoding(int _inbits) :
    LUT(NULL)
{
     inbits = _inbits;
     int effectiveBits = getNearUpperPowerOf2(inbits , addbits);
     outbits = inbits + addbits;

     LUT = new uint32_t[1 << outbits];




}

uint32_t ExtensiveCoding::encode(uint32_t input)
{
    return input;
}

uint32_t ExtensiveCoding::decode(uint32_t input)
{
    return input;
}

ExtensiveCoding::~ExtensiveCoding()
{
    delete_safe(LUT);
}

} // namespace corecvs
