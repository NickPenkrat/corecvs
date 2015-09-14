/**
 * \file main_test_sig_epsilon.cpp
 * \brief This is the main file for the test sig_epsilon
 *
 * \date Aug 21, 2010
 * \author alexander
 *
 * \ingroup autotest
 */

#include <stdio.h>
#include <iostream>
#include "gtest/gtest.h"

#include "epsilons.h"

using namespace std;

/**
 *  Test for valgrind to check boundaries problems
 **/
TEST(SigEpsilon, testEpsilonTable)
{
    LinearFeaturingParameters params(1,8,2,150);
    EpsilonTable3Linear epsTable(params);

    unsigned sum = 0;
    for (unsigned i = 0; i < 150 * 150 * 5; i++)
    {
        sum += epsTable.epsilon(i);
    }

    cout << sum << endl;
}

#ifdef WITH_SSE
/* Test for checking the SSE epsilon generation*/
TEST(SigEpsilon, _testEpsilonSSESimulated)
{
    LinearFeaturingParameters params(1,8,2,150);
    EpsilonCalcualtor3Linear epsTable(params);

    for (unsigned i = 0; i < 150 * 150 * 3; i++)
    {
        ALIGN_DATA(16) uint32_t sseResultMem[4];
        __m128i sseInput = _mm_set1_epi32 ((int32_t) i);
        __m128i sseResult = epsTable.epsilon_sse(sseInput);
        _mm_store_si128((__m128i*)&sseResultMem[0], sseResult);
#ifdef ASSERTS
        unsigned result = epsTable.epsilon(i);
        ASSERT_TRUE_P(result == sseResultMem[0], (" Simulated SSE broken %d %d -> %d\n", i, result, sseResultMem[0]));
#endif
    }
}
#endif // #ifdef WITH_SSE

#ifdef WITH_SSE
/* Test for checking the SSE epsilon generation*/
ALIGN_STACK_SSE
TEST(SigEpsilon, _testEpsilonSSEVectorised)
{
    LinearFeaturingParameters params(1,8,2,150);
    EpsilonCalcualtor3Linear epsTable(params);
    EpsilonCalcualtor3Linear epsVector(params);


    for (unsigned i = 0; i < 150 * 150 * 3; i++)
    {
        ALIGN_DATA(16) int32_t sseResultMem[4];
        __m128i sseInput = _mm_set1_epi32 ((int32_t) i);
        __m128i sseResult = epsVector.epsilon_sse(sseInput);
        _mm_store_si128((__m128i*)&sseResultMem[0], sseResult);

        unsigned result = epsTable.epsilon(i);

        printf("%d %d -> %d\n", i, result, sseResultMem[0]);

    }
}
#endif // #ifdef WITH_SSE


//int main (int /*argC*/, char **/*argV*/)
//{
//#ifdef WITH_SSE
//    _testEpsilonSSESimulated ();
//    _testEpsilonSSEVectorised ();
//#endif
//    cout << "PASSED";
//    return 0;
//}
