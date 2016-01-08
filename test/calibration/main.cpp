#include <string>
#include "gtest/gtest.h"

#include "global.h"

#include <QFile>
#include <QDir>

using namespace std;

void CheckWorkFolderCalibrationTest()
{
    const char* envDir = std::getenv("TOPCON_DIR");
    if (envDir == NULL) {
        CORE_ASSERT_FAIL("The env.var. TOPCON_DIR is missed!");
    }
    QString path(envDir);
    if (!QSTR_HAS_SLASH_AT_END(path)) {
        path += PATH_SEPARATOR;
    }

    path += "data/tests/calibration/esDistOutDist.json";
    path = QDir::toNativeSeparators(path);

    if (!QFile::exists(path)) {
        cout << "The work file <" << path.toStdString() << "> is missed" << endl;
        CORE_ASSERT_FAIL("There is no test data!");
    }
}

int main(int argc, char **argv)
{
    CheckWorkFolderCalibrationTest();

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
