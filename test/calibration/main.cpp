#include <string>
#include "gtest/gtest.h"

#include "global.h"

#include <QFile>

using namespace std;

bool CheckGDriveCalibrationFolderTest()
{
    const char* dirGDrive = std::getenv("TOPCON_DIR");
    if (dirGDrive == NULL) {
        cout << "The env.var. TOPCON_DIR is missed" << endl;
        return false;
    }
    string path = dirGDrive;
    if (!STR_HAS_SLASH_AT_END(path)) {
        path += PATH_SEPARATOR;
    }

    path += "data/tests/calibration/esDistOutDist.json";

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
