#include <stdlib.h>
#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "fixedArray.h"

using namespace std;
using namespace corecvs;

bool CheckGDriveCalibrationFolderTest()
{
    const char* dirGDrive = std::getenv("TOPCON_DIR_GDRIVE");
    const char* dirRelPath = "/data/tests/calibration/";
    const char* fileName = "esDistOutDist.json";

    std::stringstream fs;
    fs << dirGDrive << dirRelPath << fileName;

    return access(fs.str().c_str(), 0) == 0;
}

int main(int argc, char **argv)
{
    if(CheckGDriveCalibrationFolderTest)
        ::testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
    else
        CORE_ASSERT_TRUE(0, "There is no test data");
        return 0;
}
