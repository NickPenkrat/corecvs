/**
 * \file morphological.cpp
 * \brief Add Comment Here
 *
 * \date Aug 11, 2011
 * \author alexander
 */

#include "morphological.h"
#include "fastKernel.h"
#include "vectorAlgebra.h"
#include "vectorTraits.h"
namespace core3vi {

/**
 *   This function is a copy of the one in G12Buffer.cpp. But this is for a reason
 **/
template<template <typename> class KernelType>
    static void process(
            G12Buffer* input [KernelType<DummyAlgebra>::inputNumber],
            G12Buffer* output[KernelType<DummyAlgebra>::inputNumber],
            const KernelType<DummyAlgebra> &kernel = KernelType<DummyAlgebra>())
{
    BufferProcessor<
        G12Buffer,
        G12Buffer,
        KernelType,
        G12BufferAlgebra
    > processor;
    processor.process(input, output, kernel);
}


G12Buffer *Morphological::erode(G12Buffer *input, G12Buffer *element, int centerX, int centerY)
{
    G12Buffer *result = new G12Buffer(input->getSize());

    ErodeKernel<DummyAlgebra> erroder(element, centerY, centerX);
    process<ErodeKernel>(&input, &result, erroder);
    return result;
}

G12Buffer *Morphological::dilate(G12Buffer *input, G12Buffer *element, int centerX, int centerY)
{
    G12Buffer *result = new G12Buffer(input->getSize());
    DilateKernel<DummyAlgebra> dilater(element, centerY, centerX);
    process<DilateKernel>(&input, &result, dilater);
    return result;
}


} //namespace core3vi

