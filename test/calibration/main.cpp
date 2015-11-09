#include <stdlib.h>
#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "fixedArray.h"
#include <QFile>
#include <QDir>

using namespace std;
using namespace corecvs;

bool CheckGDriveCalibrationFolderTest()
{
    const char* dirGDrive = std::getenv("TOPCON_DIR_GDRIVE");
    const char* dirRelPath = "/data/tests/calibration/";
    const char* fileName = "esDistOutDist.json";

    std::stringstream fs;
    fs << dirGDrive << dirRelPath << fileName;

    std::string path = QDir::toNativeSeparators(QString(fs.str().c_str())).toStdString();

    printf("[%s] exists: %d",path.c_str(),QFile::exists(path.c_str()));
    return access(fs.str().c_str(), 0) == 0;
}

int main(int argc, char **argv)
{

    CheckGDriveCalibrationFolderTest();
          return 0;
//    ::testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();
}
