/**
 * \file main_test_jit.cpp
 * \brief This is the main file for the test jit 
 *
 * \date сен 02, 2016
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "gtest/gtest.h"

#include "global.h"


using namespace std;

TEST(jit, testjit)
{
    cout << "Starting test <jit>" << endl;
    cout << "This test is x64 and GCC only" << endl;

#if defined (__GNUC__) && __x86_64

    double sin_a, cos_a, a = 0.5;
    asm ("fldl %2;"
         "fsincos;"
         "fstpl %1;"
         "fstpl %0;" : "=m"(sin_a), "=m"(cos_a) : "m"(a));
    printf("sin(29°) = %f, cos(29°) = %f\n", sin_a, cos_a);



#endif


    cout << "Test <jit> PASSED" << endl;
}
