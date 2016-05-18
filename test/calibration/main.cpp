#include <string>
#include "gtest/gtest.h"

#include "global.h"
#include "utils.h"

#include <QFile>
#include <QDir>

void CheckWorkFolderCalibrationTest()
{
    std::string path = corecvs::HelperUtils::getFullPath("TOPCON_DIR", "data/tests/calibration/", "esDistOutDist.json");

    if (!QFile::exists(path.c_str())) {
        std::cout << "The work file <" << path << "> is missed" << std::endl;
        CORE_ASSERT_FAIL("There is no test data!");
    }
}

int main(int argc, char **argv)
{
    CheckWorkFolderCalibrationTest();

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
