/**
 * \file main_test_commandline.cpp
 * \brief This is the main file for the test commandline 
 *
 * \date авг. 21, 2015
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "global.h"
#include "commandLineSetter.h"

using namespace std;
using namespace corecvs;

void testCommandLine()
{
    cout << "testCommandLine()" << endl;
    const char *argv[] = {"--bool", "--int=42", "--double=3.14", "--string=test1"};
    int argc = CORE_COUNT_OF(argv);


    CommandLineSetter setter(argc, argv);
    bool   b = setter.getBool("bool");
    int    i = setter.getInt("int");
    double d = setter.getDouble("double");



    cout << "Bool:"   << b << endl;
    cout << "Int:"    << i << endl;
    cout << "Double:" << d << endl;

    ASSERT_TRUE(b == true, "Bool parsing problem");
    ASSERT_TRUE(i == 42  , "Int parsing problem");
    ASSERT_TRUE(d == 3.14, "Double parsing problem");
}

int main (int /*argC*/, char **/*argV*/)
{
    testCommandLine();
    cout << "PASSED" << endl;
    return 0;
}
