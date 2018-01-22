#include "core/math/extensiveCoding.h"
#include "core/math/mathUtils.h"

namespace corecvs {

ExtensiveCoding::ExtensiveCoding(int inbits)
    : mLUT(NULL)
{
    mInbits = inbits;
    getNearUpperPowerOf2(inbits, mAddbits);
    mOutbits = mInbits + mAddbits;

    mLUT = new uint32_t[(int)(1 << mOutbits)];
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
