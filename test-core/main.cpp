#include <stdlib.h>
#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "vectorOperations.h"
#include "preciseTimer.h"
#include "fixedVector.h"
#include "fixedArray.h"
#include "vector3d.h"


using namespace std;
using namespace corecvs;


TEST(EnvTest, CheckTopconDirTest) {
    if(const char* dir = std::getenv("TOPCON_DIR"))
        SUCCEED();
    else
        FAIL();
}

TEST(EnvTest, CheckTopconDirGDriveTest) {
    if(const char* dir = std::getenv("TOPCON_DIR_GDRIVE"))
        SUCCEED();
    else
        FAIL();
}

TEST(VectorTest, MulAllElements) {
    const int LENGTH = 8;
    FixedArray<int> arr(LENGTH);
    ASSERT_EQ(arr.size(), LENGTH);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
