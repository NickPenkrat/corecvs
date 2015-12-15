#include <string>
#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include <QFile>

using namespace std;

bool CheckGDriveCalibrationFolderTest()
{
    const char* dirGDrive = std::getenv("TOPCON_DIR");
    const char* dirRelPath = "/data/tests/calibration/";
    const char* fileName = "esDistOutDist.json";

    if (dirGDrive == NULL) {
        cout << "The envvar TOPCON_DIR_GDRIVE is missed" << endl;
        return false;
    }

    std::stringstream fs;
    fs << dirGDrive << dirRelPath << fileName;
    string path = fs.str();

    if (!QFile::exists(path.c_str())) {
        cout << "The file <" << path << "> is missed" << endl;
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    if (!CheckGDriveCalibrationFolderTest())
    {
          CORE_ASSERT_TRUE(0, "There is no test data");
          return 0;
    }
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
