#include <stdlib.h>
#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "fixedArray.h"

using namespace corecvs;

static bool checkFileExist(const std::string& dirPath, const std::string& filePath)
{
    std::string path = dirPath;
    if (!STR_HAS_SLASH_AT_END(path)) {
        path += PATH_SEPARATOR;
    }
    path += filePath;
    std::cout << "checking for the file <" << path << ">" << std::endl;
    return access(path.c_str(), 0) == 0;
}

static bool checkFileExist(const char* dirName, const std::string& filePath)
{
    cchar* dir = getenv(dirName);
    if (dir == NULL) {
        //FAIL();
        CORE_ASSERT_FAIL_P(("Missed environment variable %s", dirName));
        return false;
    }
    //SUCCEED();
    std::cout << dirName << "=" << dir << std::endl;

    return checkFileExist(std::string(dir), filePath);
}

// Check existance for the "./data/pair/image0001_c0.pgm"
TEST(EnvTest, CheckCurrentDirTest)
{
    std::string filePath = std::string("data")
        + PATH_SEPARATOR + "pair"
        + PATH_SEPARATOR + "image0001_c0.pgm";

    CORE_ASSERT_TRUE(checkFileExist(std::string("."), filePath), "Missed expected repo DB at the current folder");
}

// Check existance for the "<TOPCON_DIR>data/Measure_12_Roof/mosk_test_001_good/_DSC8173.jpg"
TEST(EnvTest, CheckTopconDirTest)
{
    std::string filePath = std::string("data")
        + PATH_SEPARATOR + "Measure_12_Roof"
        + PATH_SEPARATOR + "mosk_test_001_good"
        + PATH_SEPARATOR + "_DSC8173.jpg";

    CORE_ASSERT_TRUE(checkFileExist("TOPCON_DIR", filePath), "Missed expected DB at the TOPCON_DIR folder");
}

// Check existance for the "<TOPCON_DIR_GDRIVE>data/tests/SPA3_0deg_3.jpg"
TEST(EnvTest, CheckTopconDirGDriveTest)
{
    std::string filePath = std::string("data")
        + PATH_SEPARATOR + "tests"
        + PATH_SEPARATOR + "SPA3_0deg_3.jpg";

    CORE_ASSERT_TRUE(checkFileExist("TOPCON_DIR_GDRIVE", filePath), "Missed expected DB at the TOPCON_DIR_GDRIVE folder");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
