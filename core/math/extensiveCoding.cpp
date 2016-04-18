#include "extensiveCoding.h"
#include "mathUtils.h"

namespace corecvs {

ExtensiveCoding::ExtensiveCoding(int inbits)
    : mLUT(NULL)
{
    mInbits = inbits;
    getNearUpperPowerOf2(inbits, mAddbits);
    mOutbits = mInbits + mAddbits;

    mLUT = new uint32_t[1 << mOutbits];
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
    delete_safe(mLUT);
}

} // namespace corecvs
