#include "gtest/gtest.h"

#include <stdlib.h>
#include <iostream>

#include "global.h"

#include "vectorOperations.h"
#include "preciseTimer.h"
#include "fixedVector.h"
#include "fixedArray.h"

//#include <opencv2/opencv.hpp>

using namespace std;
using namespace corecvs;

//TEST(VectorTest, FailAccessTest) {
//    const int LENGTH = 8;
//    FixedArray<int> arr(LENGTH);
//    arr[8];
//    ASSERT_EQ(arr.size(), LENGTH - 1);
//}

TEST(VectorTest, Size) {
    const int LENGTH = 8;
    FixedArray<int> arr(LENGTH);
    for (int i = 0; i < LENGTH; i++)
    {
        arr[i] = i + 1;
    }
    ASSERT_EQ(arr.mulAllElements(), 40320);
}

TEST(VectorTest, MulAllElements) {
    const int LENGTH = 8;
    FixedArray<int> arr(LENGTH);
    ASSERT_EQ(arr.size(), LENGTH);
}

//TEST(OpenCvTest, OkTest) {
//    cv::Mat m;
//}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
