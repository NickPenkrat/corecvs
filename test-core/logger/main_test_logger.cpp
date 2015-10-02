/**
 * \file main_test_logger.cpp
 * \brief This is the main file for the test logger 
 *
 * \date Sep 26, 2012
 * \author alexander
 *
 * \ingroup autotest  
 */

#ifndef TRACE
#define TRACE
#endif

#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "log.h"


using namespace std;

TEST(Logger, testDummy)  // TODO: add log to file and then check its content!
{
 //   L_ERROR("Test with place");

  //  Log().info() << "Hello";

    Log logger;
    Log::MessageScoped(&logger, Log::LEVEL_ERROR, "A", 0, "F");

    cout << Log::formatted("Here we go %d\n", 1, 2, "three");
}
