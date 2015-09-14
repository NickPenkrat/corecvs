/**
 * \file aLowCodec.h
 * \brief Add Comment Here
 *
 * \date Sep 1, 2015
 * \author sfedorenko
 */
#pragma once

#include <stdint.h>
#include "rgb24Buffer.h"

using namespace corecvs;

class ALowCodec
{
public:
    ALowCodec() {}

    RGB24Buffer  *  code(const RGB24Buffer *rgb24buffer);
    RGB24Buffer  *decode(const RGB24Buffer *rgb24buffer);
};
