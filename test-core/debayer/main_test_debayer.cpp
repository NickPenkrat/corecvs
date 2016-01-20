/**
* \file main_test_debayer.cpp
* \brief This is the main file for debayer color test
*/
#include <sstream>
#include <iostream>
#include "gtest/gtest.h"

#include <global.h>
#include "g12Buffer.h"
#include "bmpLoader.h"
#include "ppmLoader.h"
#include "converters/debayer.h"
#include "converters/errorMetrics.h"

using namespace std;
using namespace corecvs;


Vector2d<int> redRegion(0, 0);
Vector2d<int> greenRegion(180, 0);
Vector2d<int> blueRegion(90, 0);

int regionSize = 50;

void performTest(int64_t &r_sum, int64_t &g_sum, int64_t &b_sum, G12Buffer* ppm, RGB48Buffer *result)
{

    for (int x = redRegion.x(); x < redRegion.x() + regionSize; x++)
        for (int y = redRegion.y(); y < redRegion.y() + regionSize; y++)
        {
            r_sum += (2 * result->element(y, x).r() - result->element(y, x).g() - result->element(y, x).b());
            g_sum += (2 * result->element(y, x).g() - result->element(y, x).r() - result->element(y, x).b());
            b_sum += (2 * result->element(y, x).b() - result->element(y, x).r() - result->element(y, x).g());
        }

    CORE_ASSERT_TRUE(r_sum > b_sum + g_sum, "The decoded image has a wrong red channel");
    r_sum = 0, g_sum = 0, b_sum = 0;

    for (int x = greenRegion.x(); x < greenRegion.x() + regionSize; x++)
        for (int y = greenRegion.y(); y < greenRegion.y() + regionSize; y++)
        {
            r_sum += (2 * result->element(y, x).r() - result->element(y, x).g() - result->element(y, x).b());
            g_sum += (2 * result->element(y, x).g() - result->element(y, x).r() - result->element(y, x).b());
            b_sum += (2 * result->element(y, x).b() - result->element(y, x).r() - result->element(y, x).g());
        }

    CORE_ASSERT_TRUE(g_sum > r_sum + b_sum, "The decoded image has a wrong red channel");
    r_sum = 0, g_sum = 0, b_sum = 0;
    
    for (int x = blueRegion.x(); x < blueRegion.x() + regionSize; x++)
        for (int y = blueRegion.y(); y < blueRegion.y() + regionSize; y++)
        {
            r_sum += (2 * result->element(y, x).r() - result->element(y, x).g() - result->element(y, x).b());
            g_sum += (2 * result->element(y, x).g() - result->element(y, x).r() - result->element(y, x).b());
            b_sum += (2 * result->element(y, x).b() - result->element(y, x).r() - result->element(y, x).g());
        }
    CORE_ASSERT_TRUE(b_sum > r_sum + g_sum, "The decoded image has a wrong blue channel");
}

TEST(Debayer, colorTestYchannel) 
{
    FILE* out = fopen("out.txt", "w");
    fprintf(out, "|_.#|_.RMSD|_.maxabs|\n");
    for (int i = 1; i <= 10; i++) {
        PPMLoader *ppmLoader = new PPMLoader();
        //G12Buffer *ppm = ppmLoader->load("data/testdata/cm_star.pgm");
        G12Buffer *ppm = ppmLoader->load("C:/pgmdata/" + to_string(i) + ".pgm");
        G12Buffer *y = new G12Buffer(ppm->getSize());
        G12Buffer *y2 = new G12Buffer(ppm->getSize());

        //Debayer d(ppm);
        //RGB48Buffer* result = new RGB48Buffer(ppm->h, ppm->w, false);
        //d.toRGB48(Debayer::AHD, result);
        RGB48Buffer* result = ppmLoader->loadRGB("C:/pgmtruth/" + to_string(i) + ".ppm", nullptr);
        int max = 0;
        Vector2d<int> maxpos;
        double errint = 0;
        double error = ErrorMetrics::Yrmsd(ppm, result, 3, 8, y, y2, max, maxpos, errint);

        // TODO: clarify issue with Y-channel extraction yielding results of different brightness
        printf("RMSD between Y-channel images: %.3lf", error);
        CORE_ASSERT_TRUE(error < 7, "RMSE is too high!");

        delete_safe(ppm);
        delete_safe(ppmLoader);
        delete_safe(result);

        fprintf(out, "|%d|%.3lf|%.3lf|%.4lf\%|%d|(%d, %d)|\n", i, error, errint, abs(error-errint)/std::max(error,errint)*100, max, maxpos[maxpos.FIELD_X], maxpos[maxpos.FIELD_Y]);

        BMPLoader().save("y_orig_" + to_string(i) + ".bmp", y);
        BMPLoader().save("y_debayer_" + to_string(i) + ".bmp", y2);

    }
    fclose(out);
}

TEST(Debayer, colorTestNearest)
{
    PPMLoader *ppmLoader = new PPMLoader();
    G12Buffer *ppm = ppmLoader->load("data/testdata/test_debayer.pgm");
    CORE_ASSERT_TRUE(ppm != NULL, "PPM Image load failed");
    CORE_ASSERT_TRUE(ppm->verify(), "PPM Image verification failed");
    Debayer d(ppm);
    RGB48Buffer* result = new RGB48Buffer(ppm->h, ppm->w, false);
    d.toRGB48(Debayer::Nearest, result);

    int64_t r_sum = 0, g_sum = 0, b_sum = 0;
    performTest(r_sum, g_sum, b_sum, ppm, result);

    delete_safe(ppm);
    delete_safe(ppmLoader);
    delete_safe(result);
}

TEST(Debayer, colorTestBilinear)
{
    PPMLoader *ppmLoader = new PPMLoader();
    G12Buffer *ppm = ppmLoader->load("data/testdata/test_debayer.pgm");
    CORE_ASSERT_TRUE(ppm != NULL, "PPM Image load failed");
    CORE_ASSERT_TRUE(ppm->verify(), "PPM Image verification failed");
    Debayer d(ppm);
    RGB48Buffer* result = new RGB48Buffer(ppm->h, ppm->w, false);
    d.toRGB48(Debayer::Bilinear, result);

    int64_t r_sum = 0, g_sum = 0, b_sum = 0;
    performTest(r_sum, g_sum, b_sum, ppm, result);

    delete_safe(ppm);
    delete_safe(ppmLoader);
    delete_safe(result);
}

TEST(Debayer, colorTestAHD)
{
    PPMLoader *ppmLoader = new PPMLoader();
    G12Buffer *ppm = ppmLoader->load("data/testdata/test_debayer.pgm");
    CORE_ASSERT_TRUE(ppm != NULL, "PPM Image load failed");
    CORE_ASSERT_TRUE(ppm->verify(), "PPM Image verification failed");
    Debayer d(ppm);
    RGB48Buffer* result = new RGB48Buffer(ppm->h, ppm->w, false);
    d.toRGB48(Debayer::AHD, result);

    int64_t r_sum = 0, g_sum = 0, b_sum = 0;
    performTest(r_sum, g_sum, b_sum, ppm, result);

    delete_safe(ppm);
    delete_safe(ppmLoader);
    delete_safe(result);
}
