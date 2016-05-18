/**
 * \file main_test_filesystem.cpp
 * \brief This is the main file for the test filesystem
 *
 * \date Mar 11, 2016
 * \author sf
 * \ingroup autotest
 */
#include <iostream>
#include "gtest/gtest.h"

#include <global.h>

#include "folderScanner.h"

using namespace corecvs;

TEST(Filesystem, test1)
{
    const string pathDir = ".";
    vector<string> childs;

    bool res = FolderScanner::scan(pathDir, childs);
    CORE_ASSERT_TRUE_S(res);

    bool isOk = false;

    std::cout << "testFilesystem:: current folder has " << childs.size() << " files:" << std::endl;
    for (string& child : childs)
    {
        std::cout << child << std::endl;

        if (child == pathDir + PATH_SEPARATOR + "main.cpp" ||
            child == pathDir + PATH_SEPARATOR + "cvs.pro")
            isOk = true;
    }

    CORE_ASSERT_TRUE(isOk, "current folder must have one of the files: main.cpp or cvs.pro!");
}
