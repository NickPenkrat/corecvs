#include <stdlib.h>
#include <iostream>
#include "gtest/gtest.h"

#include "core/utils/global.h"

#include "core/math/vector/fixedArray.h"

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

// Check existance for the "./data/pair/image0001_c0.pgm"
TEST(EnvTest, CheckCurrentDirTest)
{
    std::string filePath = std::string("data")
        + PATH_SEPARATOR + "pair"
        + PATH_SEPARATOR + "image0001_c0.pgm";

    if (!checkFileExist(std::string("."), filePath))
    {
        std::cout << "file: " << filePath.c_str() << " is missed" << std::endl;
        return;
    }
    std::cout << "found repo DB at the current folder" << std::endl;
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
