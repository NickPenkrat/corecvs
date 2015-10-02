/**
 * \file main_test_fastkernel_double.cpp
 * \brief This is the main file for the test fastkernel_double
 *
 * \date авг. 22, 2015
 * \author alexander
 *
 * \ingroup autotest
 */

#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "preciseTimer.h"
#include "chessBoardCornerDetector.h"
#include "fastKernel.h"
#include "sobel.h"
#include "vectorTraits.h"
#include "arithmetic.h"
#include "copyKernel.h"

using corecvs::SobelVerticalKernel;
using corecvs::ScalarAlgebraDouble;

using namespace std;
using namespace corecvs;

const static unsigned POLUTING_INPUTS = 10;
const static unsigned LIMIT = 50;

const static int  TEST_H_SIZE = 1000;
const static int  TEST_W_SIZE = 1500;

/*const static int  TEST_H_SIZE = 8;
const static int  TEST_W_SIZE = 8;*/


template<typename Type = uint16_t>
class VisiterSemiRandom
{
public:
    void operator() (int y , int x , Type &element) {
        element = Type(((unsigned)(y * 54536351 + x * 8769843)));

        /*element = Type(1.0);
        if (y == 3 && x == 3) {
            element = Type(0.0);
        }*/
    }
};

/**
 *  Cool new style (FastKernels) Vertical Sobel filter
 **/
template <typename Algebra>
class VerticalEdgeKernel
{
public:
    static const int inputNumber = 1;
    static const int outputNumber = 1;

    inline static int getCenterX(){ return 1; }
    inline static int getCenterY(){ return 1; }
    inline static int getSizeX()  { return 3; }
    inline static int getSizeY()  { return 3; }

    typedef typename Algebra::InputType       Type;
    typedef typename Algebra::InputSignedType SignedType;

template <typename OtherAlgebra>
    VerticalEdgeKernel(const VerticalEdgeKernel<OtherAlgebra> & /*other*/){}

    VerticalEdgeKernel() {}


    inline void process(Algebra &algebra) const
    {
        Type a00 = algebra.getInput(0,0);
        Type a10 = algebra.getInput(1,0);
        Type a20 = algebra.getInput(2,0);
        Type a02 = algebra.getInput(0,2);
        Type a12 = algebra.getInput(1,2);
        Type a22 = algebra.getInput(2,2);

        SignedType negative = SignedType(a00 + a10 + a20);
        SignedType positive = SignedType(a02 + a12 + a22);

        Type result = Type(positive - negative);
        algebra.putOutput(0,0,result);

        /*Type a00 = algebra.getInput(0,0);
        algebra.putOutput(0,0, a00);*/
    }
};



TEST(FastKernelDouble, testDoubleConvolve)  // TODO: move to perf-tests
{
    DpImage * input[POLUTING_INPUTS];

    VisiterSemiRandom<DpImage::InternalElementType> vis;
    for (unsigned i = 0; i < POLUTING_INPUTS; i++)
    {
        input[i] = new DpImage(TEST_H_SIZE ,TEST_W_SIZE);
        input[i]->touchOperationElementwize(vis);
    }

    DpImage *outputScalar[POLUTING_INPUTS];
    DpImage *outputSimple[POLUTING_INPUTS];
    DpImage *outputVector[POLUTING_INPUTS];
    DpImage *outputKernalized[POLUTING_INPUTS];

    for (unsigned i = 0; i < POLUTING_INPUTS; i++)
    {
        outputSimple[i]     = new DpImage(TEST_H_SIZE ,TEST_W_SIZE);
        outputScalar[i]     = new DpImage(TEST_H_SIZE ,TEST_W_SIZE);
        outputVector[i]     = new DpImage(TEST_H_SIZE ,TEST_W_SIZE);
        outputKernalized[i] = new DpImage(TEST_H_SIZE ,TEST_W_SIZE);
    }

    SYNC_PRINT(("Profiling Vertical edge detection:\n"));
    SYNC_PRINT(("   We will %d times pump [%dx%d] buffers with double elements (%lu bytes):\n", LIMIT, TEST_H_SIZE, TEST_W_SIZE, sizeof(double) ));
    uint64_t bytesr = LIMIT * TEST_H_SIZE * TEST_W_SIZE * sizeof(double);
    uint64_t bytesw = bytesr;
    SYNC_PRINT(("   Totally read  - %" PRIu64 " bytes = %.2f Mb \n", bytesr, bytesr / 1000000.0));
    SYNC_PRINT(("   Totally write - %" PRIu64 " bytes = %.2f Mb \n", bytesw, bytesw / 1000000.0));

    PreciseTimer start;
    /* Run Simple */
        /* Preparation */
        double duk[] = {-1.0,  0.0,  1.0, -1.0, 0.0, 1.0, -1.0, 0.0, 1.0};
        DpKernel K (3, 3, duk);


        SYNC_PRINT(("Profiling     Simple Approach:"));
        start = PreciseTimer::currentTime();
        for (unsigned i = 0; i < LIMIT; i++) {
            /*delete_safe(outputSimple[i % POLUTING_INPUTS]);
            outputSimple[i % POLUTING_INPUTS] = input[i % POLUTING_INPUTS]->doConvolve<DpImage>(&K);*/
            input[i % POLUTING_INPUTS]->doConvolve<DpImage>(outputSimple[i % POLUTING_INPUTS], &K);
        }
        uint64_t delaySimple = start.usecsToNow();
        SYNC_PRINT(("%8" PRIu64 "us %8" PRIu64 "ms SP: %8" PRIu64 "us\n", delaySimple, delaySimple / 1000, delaySimple / LIMIT));

    /* Run kernalized */

#if defined (WITH_SSE)
        SYNC_PRINT(("Profiling Kernalized Approach:"));
        start = PreciseTimer::currentTime();

        ConvolveKernel<DummyAlgebra> convolver(&K, 1, 1);
        BufferProcessor<DpImage, DpImage, ConvolveKernel, VectorAlgebraDouble> procScalar;
        for (unsigned i = 0; i < LIMIT; i++) {
            procScalar.process(&input[i % POLUTING_INPUTS], &outputKernalized[i % POLUTING_INPUTS], convolver);
        }
        uint64_t delayKernalized = start.usecsToNow();
        SYNC_PRINT(("%8" PRIu64 "us %8" PRIu64 "ms SP: %8" PRIu64 "us\n", delayKernalized, delayKernalized / 1000, delayKernalized / LIMIT));
#endif

    /* Run scalar */
    VerticalEdgeKernel<DummyAlgebra> kernel;

        SYNC_PRINT(("Profiling     Scalar Approach:"));
        start = PreciseTimer::currentTime();
        BufferProcessor<DpImage, DpImage, VerticalEdgeKernel, ScalarAlgebraDouble> procKernalized;
        for (unsigned i = 0; i < LIMIT; i++) {
            DpImage *cinput[1]  = { input       [i % POLUTING_INPUTS] };
            DpImage *coutput[1] = { outputScalar[i % POLUTING_INPUTS] };

            procKernalized.process(cinput, coutput, kernel);
        }
        uint64_t delayScalar = start.usecsToNow();
        SYNC_PRINT(("%8" PRIu64 "us %8" PRIu64 "ms SP: %8" PRIu64 "us\n", delayScalar, delayScalar / 1000, delayScalar / LIMIT));


    /* Run vector */
#if defined (WITH_SSE)
    CopyKernel<DummyAlgebra> kernel1;
        SYNC_PRINT(("Profiling     Vector Approach:"));
        start = PreciseTimer::currentTime();
        BufferProcessor<DpImage, DpImage, CopyKernel, VectorAlgebraDouble> procVector;
        for (unsigned i = 0; i < LIMIT; i++) {
            DpImage *cinput[1]  = { input       [i % POLUTING_INPUTS] };
            DpImage *coutput[1] = { outputVector[i % POLUTING_INPUTS] };

            procVector.process(cinput, coutput, kernel1);
        }
        uint64_t delayVector = start.usecsToNow();
        SYNC_PRINT(("%8" PRIu64 "us %8" PRIu64 "ms SP: %8" PRIu64 "us\n", delayVector, delayVector / 1000, delayVector / LIMIT));

#endif

    /* Report bandwidth estimation*/



#if defined (WITH_SSE)
    SYNC_PRINT(("Kernalized Bandwidth used is %.2lf bytes/s = %.2lf Mb/s = %.2lf Gb/s \n",
                (double)(bytesr + bytesw) / delayKernalized * 1000000.0,
                (double)(bytesr + bytesw) / delayKernalized,
                (double)(bytesr + bytesw) / delayKernalized / 1000.0
               ));

    SYNC_PRINT(("Scalar     Bandwidth used is %.2lf bytes/s = %.2lf Mb/s = %.2lf Gb/s \n",
                (double)(bytesr + bytesw) / delayScalar * 1000000.0,
                (double)(bytesr + bytesw) / delayScalar,
                (double)(bytesr + bytesw) / delayScalar / 1000.0
               ));

    SYNC_PRINT(("Vector     Bandwidth used is %.2lf bytes/s = %.2lf Mb/s = %.2lf Gb/s \n",
                (double)(bytesr + bytesw) / delayVector * 1000000.0,
                (double)(bytesr + bytesw) / delayVector,
                (double)(bytesr + bytesw) / delayVector / 1000.0
               ));
#endif

    /* Print results*/
     if (TEST_H_SIZE < 15 && TEST_W_SIZE < 15)
     {
         cout << *input[0] << endl;

         cout << "Simple result:" << endl;
         cout << *outputSimple[0] << endl;
         cout << "Scalar result:" << endl;
         cout << *outputScalar[0] << endl;
#if defined (WITH_SSE)
         cout << "Vector result:" << endl;
         cout << *outputVector[0] << endl;
#endif
     }


    /* Check equality */
#if defined (WITH_SSE) || defined (WITH_AVX)
     for (int i = 0; i < TEST_H_SIZE; i++)
     {
         for (int j = 0; j < TEST_W_SIZE; j++)
         {
             CORE_ASSERT_DOUBLE_EQUAL_EP(outputScalar[0]->element(i,j),outputVector[0]->element(i,j), 1e-5,
                     ("Scalar and Vector computation results are not equal at (%d, %d) (%lf vs %lf).\n",
                      i,j, outputScalar[0]->element(i,j),outputVector[0]->element(i,j)));
         }
     }


     for (int i = 0; i < TEST_H_SIZE; i++)
     {
         for (int j = 0; j < TEST_W_SIZE; j++)
         {
             CORE_ASSERT_DOUBLE_EQUAL_EP(outputScalar[0]->element(i, j), outputKernalized[0]->element(i, j), 1e-5,
                     ("Scalar and Vector computation results are not equal at (%d, %d) (%lf vs %lf).\n",
                      i,j, outputScalar[0]->element(i,j),outputKernalized[0]->element(i,j)));
         }
     }
#endif

     for (int i = 1; i < TEST_H_SIZE - 1; i++)
     {
         for (int j = 1; j < TEST_W_SIZE - 1; j++)
         {
             CORE_ASSERT_DOUBLE_EQUAL_EP(outputScalar[0]->element(i, j), outputSimple[0]->element(i, j), 1e-5,
                     ("Scalar and Simple computation results are not equal at (%d, %d) (%lf vs %lf).\n",
                      i,j, outputScalar[0]->element(i,j),outputSimple[0]->element(i,j)));
         }
     }


    /* Cleanup */
    for (unsigned i = 0; i < POLUTING_INPUTS; i++)
        delete_safe(input[i]);

    for (unsigned i = 0; i < POLUTING_INPUTS; i++)
    {
        delete_safe(outputScalar[i]);
        delete_safe(outputVector[i]);
        delete_safe(outputKernalized[i]);
        delete_safe(outputSimple[i]);
    }
}


TEST(FastKernelDouble, testLargeKernel)  // TODO: move to perf-tests
{
    DpImage * input[POLUTING_INPUTS];
    PreciseTimer start;

    VisiterSemiRandom<DpImage::InternalElementType> vis;
    for (unsigned i = 0; i < POLUTING_INPUTS; i++)
    {
        input[i] = new DpImage(TEST_H_SIZE, TEST_W_SIZE);
        input[i]->touchOperationElementwize(vis);
    }

    DpImage *outputKernalized[POLUTING_INPUTS];

    for (unsigned i = 0; i < POLUTING_INPUTS; i++)
    {
        outputKernalized[i] = new DpImage(TEST_H_SIZE, TEST_W_SIZE);
    }

    for (int size = 3; size < 20; size += 4)
    {
        DpImage *kernel = new DpImage(size, size);
        kernel->touchOperationElementwize(vis);

#if defined (WITH_SSE)
            SYNC_PRINT(("Profiling Kernalized Approach [%dx%d]:", kernel->w, kernel->h));
            start = PreciseTimer::currentTime();

            ConvolveKernel<DummyAlgebra> convolver(kernel, kernel->h / 2, kernel->w / 2);
            BufferProcessor<DpImage, DpImage, ConvolveKernel, VectorAlgebraDouble> procScalar;
            for (unsigned i = 0; i < LIMIT; i++) {
                procScalar.process(&input[i % POLUTING_INPUTS], &outputKernalized[i % POLUTING_INPUTS], convolver);
            }
            uint64_t delayKernalized = start.usecsToNow();
            SYNC_PRINT(("%8" PRIu64 "us %8" PRIu64 "ms SP: %8" PRIu64 "us\n", delayKernalized, delayKernalized / 1000, delayKernalized / LIMIT));
#endif

        delete_safe(kernel);
    }

    for (unsigned i = 0; i < POLUTING_INPUTS; i++)
        delete_safe(input[i]);

    for (unsigned i = 0; i < POLUTING_INPUTS; i++)
    {
        delete_safe(outputKernalized[i]);
    }
}
