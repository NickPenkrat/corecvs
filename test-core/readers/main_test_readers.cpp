/**
 * \file main_test_readers.cpp
 * \brief This is the main file for the test readers 
 *
 * \date авг. 25, 2015
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "global.h"
#include "readers.h"
#include "rgbColor.h"

using namespace std;
using namespace corecvs;

void testRGBReaders()
{
    cout << "testRGBReaders()" << endl;
    RGBColor data[16] = {RGBColor::Yellow(), RGBColor::Indigo(), RGBColor::Violet(), RGBColor::Cyan() };

    FixedVector<Int32x4, 2> color = SSEReader2BBBB_QQQQ::read((uint8_t *)data);

    cout << color[0][0] << endl;
    cout << color[0][1] << endl;
    cout << color[0][2] << endl;

    cout << color[1][0] << endl;
    cout << color[1][1] << endl;
    cout << color[1][2] << endl;

    cout << data[0] << " " << data[1] << endl;


}

int main (int /*argC*/, char **/*argV*/)
{
    testRGBReaders();
    cout << "PASSED" << endl;
    return 0;
}
