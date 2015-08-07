/**
 * \file main_test_yuv.cpp
 * \brief This is the main file for the test yuv 
 *
 * \date июля 28, 2015
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "global.h"

#include "yuv/yuv422Planes.h"


using namespace std;
using corecvs::YUV422Planes;
using corecvs::YUVColor;


void testYUVConvertion( void )
{
    cout << "testYUVConvertion(): called" << endl;

    RGBColor color[] = {RGBColor::Black(), RGBColor::White(), RGBColor::Red(), RGBColor::Orange(), RGBColor::Yellow(), RGBColor::Green(), RGBColor::Cyan(), RGBColor::Violet(), RGBColor::Indigo() };

    for (int i = 0; i < CORE_COUNT_OF(color); i++)
    {
        cout << i << " = rgb:" << color[i];
        YUVColor converted = YUVColor::FromRGB(color[i]);
        cout << " -> yuv:" << converted << " -> ";
        RGBColor back = converted.toRGB();
        cout << " -> rgb:" << back << endl;
    }
    return;
}

void testYUVDraw( void )
{

}


int main (int /*argC*/, char **/*argV*/)
{
    testYUVConvertion();
    cout << "PASSED" << endl;
    return 0;
}