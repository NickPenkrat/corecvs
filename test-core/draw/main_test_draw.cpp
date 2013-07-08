/**
 * \file main_test_draw.cpp
 * \brief This is the main file for the test draw 
 *
 * \date Apr 19, 2011
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>

#include "global.h"

#include "rgb24Buffer.h"
#include "bmpLoader.h"


using namespace std;
using namespace core3vi;

void testCircles(void)
{
    RGB24Buffer *buffer = new RGB24Buffer(21, 42);

    RGBColor colors[] = {
        RGBColor::Red(),
        RGBColor::Blue(),
        RGBColor::Yellow(),
        RGBColor::Black(),
        RGBColor::Indigo()
    };

    for (int i = 11; i >=1; i-= 1)
    {
        buffer->drawCircle(10,10, i, colors[i % CORE_COUNT_OF(colors)] );
    }

    for (int i = 11; i >=1; i-= 2)
    {
        buffer->drawArc(31, 10, i, colors[i % CORE_COUNT_OF(colors)] );
    }

    BMPLoader().save("circles.bmp", buffer);
    delete_safe(buffer);
}

int main (int /*argC*/, char ** /*argV*/)
{
    testCircles();
        cout << "PASSED" << endl;
        return 0;
}
