#pragma once

#include "RGB24Buffer.h"
#include "displacementBuffer.h"

void remap(corecvs::RGB24Buffer &src, corecvs::RGB24Buffer &dst, corecvs::DisplacementBuffer &transform);
