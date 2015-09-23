#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "../utils/framesources/decoders/aLowCodec.h"       // TODO: move it into future test-utils project!

using namespace corecvs;

inline void testChan(uint16_t chan12)
{
    uint8_t  coded  = code12to8(chan12);
    uint16_t res    = decode8to12(coded);

    if (chan12 <   64) { CORE_ASSERT_TRUE_S(CORE_ABS(res - chan12) <=  0); return; }
    if (chan12 <  128) { CORE_ASSERT_TRUE_S(CORE_ABS(res - chan12) <=  1); return; }
    if (chan12 <  256) { CORE_ASSERT_TRUE_S(CORE_ABS(res - chan12) <=  2); return; }
    if (chan12 <  512) { CORE_ASSERT_TRUE_S(CORE_ABS(res - chan12) <=  4); return; }
    if (chan12 < 1024) { CORE_ASSERT_TRUE_S(CORE_ABS(res - chan12) <=  8); return; }
    if (chan12 < 2048) { CORE_ASSERT_TRUE_S(CORE_ABS(res - chan12) <= 16); return; }
    if (chan12 < 4096) { CORE_ASSERT_TRUE_S(CORE_ABS(res - chan12) <= 32); return; }

    CORE_ASSERT_FAIL("Channel value is out of range");
}

inline void testRGB(int r8, int g8, int b8)
{
    testChan(r8 << 4);
    testChan(g8 << 4);
    testChan(b8 << 4);
}

inline void testDeath()
{
    printf("testDeath is called:\n");
    exit(-2);
}

TEST(ALowCodec, testALowCodec)
{
    testRGB( 20,  40,  70);
    testRGB(130, 250, 255);

    testChan(350);
    testChan(600);
    testChan(1023);
    testChan(1024);
    testChan(2100);
    testChan(4095);

    //SYNC_PRINT(("Now the assertion must be raised, but the test will be failed: "));
    //testChan(5000);

#ifdef WIN32
    ASSERT_EXIT(testDeath(), ::testing::ExitedWithCode(-2), "");
#else
    ASSERT_EXIT(testDeath(), ::testing::ExitedWithCode(256 -2), "");    // TODO: this is a bug on Linux for the gtest implementation?
#endif

    SYNC_PRINT(("Now the assertion must raised and caught: "));
    try {
        testChan(16300);
    }
    catch (const AssertException & e) {
        SYNC_PRINT(("We've got AssertException (%s)\n", e.what()));
        SUCCEED();
        return;
    }
    catch (...) {
        SYNC_PRINT(("We've got other than AssertException\n"));
    }
    ADD_FAILURE() << "Didn't throw AssertException as expected";

    //int *p = 0; *p = 345;
    //CORE_ASSERT_FAIL("testChan: check_fail");
}
