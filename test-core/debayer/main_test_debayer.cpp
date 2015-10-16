/**
* \file main_test_fileformats.cpp
* \brief This is the main file for the test fileformats
*
* \date Aug 22, 2010
* \author alexander
* \ingroup autotest
*/
#include <sstream>
#include <iostream>
#include "gtest/gtest.h"

#include <global.h>
#include "g12Buffer.h"
#include "bmpLoader.h"
#include "ppmLoader.h"
#include "converters/debayer.h"

using namespace std;
using namespace corecvs;

TEST(Debayer, colorTest)
{
    /** Test case 1 */
    PPMLoader *ppmLoader = new PPMLoader();
    G12Buffer *ppm = ppmLoader->load("data/testdata/test_debayer.pgm");
    CORE_ASSERT_TRUE(ppm != NULL, "BMP Image load failed");
    CORE_ASSERT_TRUE(ppm->verify(), "BMP Image verification failed");
    Debayer d(ppm);
    G12Buffer ** result = d.linear();
    Vector2d<int> redRegion(2, 2);
    Vector2d<int> greenRegion(180, 2);
    Vector2d<int> blueRegion(90, 2);

    int regionSize = 50;

    int64_t r_sum = 0, g_sum = 0, b_sum = 0;
    for (int x = redRegion.x(); x < redRegion.x() + regionSize; x++)
        for (int y = redRegion.y(); y < redRegion.y() + regionSize; y++)
        {
            r_sum += (2 * result[0]->element(y, x) - result[1]->element(y, x) - result[2]->element(y, x));
            g_sum += (2 * result[1]->element(y, x) - result[0]->element(y, x) - result[2]->element(y, x));
            b_sum += (2 * result[2]->element(y, x) - result[0]->element(y, x) - result[1]->element(y, x));
        }

    CORE_ASSERT_TRUE(r_sum > b_sum + g_sum, "The decoded image has a wrong red channel");
    r_sum = 0, g_sum = 0, b_sum = 0;

    for (int x = greenRegion.x(); x < greenRegion.x() + regionSize; x++)
        for (int y = greenRegion.y(); y < greenRegion.y() + regionSize; y++)
        {
            r_sum += (2 * result[0]->element(y, x) - result[1]->element(y, x) - result[2]->element(y, x));
            g_sum += (2 * result[1]->element(y, x) - result[0]->element(y, x) - result[2]->element(y, x));
            b_sum += (2 * result[2]->element(y, x) - result[0]->element(y, x) - result[1]->element(y, x));
        }

    CORE_ASSERT_TRUE(g_sum > r_sum + b_sum, "The decoded image has a wrong red channel");
    r_sum = 0, g_sum = 0, b_sum = 0;

    for (int x = blueRegion.x(); x < blueRegion.x() + regionSize; x++)
        for (int y = blueRegion.y(); y < blueRegion.y() + regionSize; y++)
        {
            r_sum += (2 * result[0]->element(y, x) - result[1]->element(y, x) - result[2]->element(y, x));
            g_sum += (2 * result[1]->element(y, x) - result[0]->element(y, x) - result[2]->element(y, x));
            b_sum += (2 * result[2]->element(y, x) - result[0]->element(y, x) - result[1]->element(y, x));
        }

    CORE_ASSERT_TRUE(b_sum > r_sum + g_sum, "The decoded image has a wrong blue channel");

    delete ppm;
    delete ppmLoader;
}
