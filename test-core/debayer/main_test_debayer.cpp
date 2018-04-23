/**
* \file main_test_debayer.cpp
* \brief This is the main file for debayer color test
*/
#include <sstream>
#include <iostream>
#include "gtest/gtest.h"

#include "core/utils/global.h"
#include "core/buffers/g12Buffer.h"
#include "core/fileformats/bmpLoader.h"
#include "core/fileformats/ppmLoader.h"
#include "core/buffers/converters/debayer.h"
#include "core/buffers/converters/errorMetrics.h"

using namespace corecvs;

Vector2d<int> redRegion(0, 0);
Vector2d<int> greenRegion(180, 0);
Vector2d<int> blueRegion(90, 0);

int regionSize = 50;

void performTest(int64_t &r_sum, int64_t &g_sum, int64_t &b_sum, RGB48Buffer *result)
{
    for (int x = redRegion.x(); x < redRegion.x() + regionSize; x++)
        for (int y = redRegion.y(); y < redRegion.y() + regionSize; y++) {
            r_sum += (2 * result->element(y, x).r() - result->element(y, x).g() - result->element(y, x).b());
            g_sum += (2 * result->element(y, x).g() - result->element(y, x).r() - result->element(y, x).b());
            b_sum += (2 * result->element(y, x).b() - result->element(y, x).r() - result->element(y, x).g());
        }

    CORE_ASSERT_TRUE(r_sum > b_sum + g_sum, "The decoded image has a wrong red channel");
    r_sum = 0, g_sum = 0, b_sum = 0;

    for (int x = greenRegion.x(); x < greenRegion.x() + regionSize; x++)
        for (int y = greenRegion.y(); y < greenRegion.y() + regionSize; y++) {
            r_sum += (2 * result->element(y, x).r() - result->element(y, x).g() - result->element(y, x).b());
            g_sum += (2 * result->element(y, x).g() - result->element(y, x).r() - result->element(y, x).b());
            b_sum += (2 * result->element(y, x).b() - result->element(y, x).r() - result->element(y, x).g());
        }

    CORE_ASSERT_TRUE(g_sum > r_sum + b_sum, "The decoded image has a wrong red channel");
    r_sum = 0, g_sum = 0, b_sum = 0;

    for (int x = blueRegion.x(); x < blueRegion.x() + regionSize; x++)
        for (int y = blueRegion.y(); y < blueRegion.y() + regionSize; y++) {
            r_sum += (2 * result->element(y, x).r() - result->element(y, x).g() - result->element(y, x).b());
            g_sum += (2 * result->element(y, x).g() - result->element(y, x).r() - result->element(y, x).b());
            b_sum += (2 * result->element(y, x).b() - result->element(y, x).r() - result->element(y, x).g());
        }
    CORE_ASSERT_TRUE(b_sum > r_sum + g_sum, "The decoded image has a wrong blue channel");
}

TEST(Debayer, colorTestYchannel)
{
    FILE* out = fopen("out.csv", "w");
    fprintf(out, "#, RMSD (double), RMSD, improvement, maxabs, coords maxabs\n");

    std::unique_ptr<G12Buffer> p(PPMLoader().loadG12("data/testdata/test_debayer.pgm"));
    CORE_ASSERT_TRUE(p != NULL, "PGM test image load failed");

    Debayer d(p.get(), 8, nullptr, 0);

    std::unique_ptr<RGB48Buffer> result(new RGB48Buffer(p->h, p->w, false));
    d.toRGB48(DebayerMethod::BILINEAR, result.get());

    double error = ErrorMetrics::Yrmsd(p.get(), result.get(), 3, 8);

    // TODO: clarify issue with Y-channel extraction yielding results of different brightness
    printf("RMSD between Y-channel images: %.3lf", error);
    CORE_ASSERT_TRUE(error < 15, "RMSE is too high!");

    fclose(out);
}

TEST(Debayer, colorTestNearest)
{
    std::unique_ptr<G12Buffer> p(PPMLoader().loadG12("data/testdata/test_debayer.pgm"));
    CORE_ASSERT_TRUE(p != NULL, "PPM Image load failed");
    CORE_ASSERT_TRUE(p->verify(), "PPM Image verification failed");

    Debayer d(p.get());
    std::unique_ptr<RGB48Buffer> result(new RGB48Buffer(p->h, p->w, false));
    d.toRGB48(DebayerMethod::NEAREST, result.get());

    int64_t r_sum = 0, g_sum = 0, b_sum = 0;
    performTest(r_sum, g_sum, b_sum, result.get());
}

TEST(Debayer, colorTestBilinear)
{
    std::unique_ptr<G12Buffer> p(PPMLoader().loadG12("data/testdata/test_debayer.pgm"));
    CORE_ASSERT_TRUE(p != NULL, "PPM Image load failed");
    CORE_ASSERT_TRUE(p->verify(), "PPM Image verification failed");
    Debayer d(p.get());
    std::unique_ptr<RGB48Buffer> result(new RGB48Buffer(p->h, p->w, false));

    d.toRGB48(DebayerMethod::BILINEAR, result.get());

    int64_t r_sum = 0, g_sum = 0, b_sum = 0;
    performTest(r_sum, g_sum, b_sum, result.get());
}

TEST(Debayer, colorTestAHD)
{
    std::unique_ptr<G12Buffer> p(PPMLoader().loadG12("data/testdata/test_debayer.pgm"));
    CORE_ASSERT_TRUE(p != NULL, "PPM Image load failed");
    CORE_ASSERT_TRUE(p->verify(), "PPM Image verification failed");
    Debayer d(p.get());
    std::unique_ptr<RGB48Buffer> result(new RGB48Buffer(p->h, p->w, false));
    d.toRGB48(DebayerMethod::AHD, result.get());

    int64_t r_sum = 0, g_sum = 0, b_sum = 0;
    performTest(r_sum, g_sum, b_sum, result.get());
}

TEST(Debayer, bayerShiftTest) 
{
    std::unique_ptr<RGB48Buffer> ppm(PPMLoader().loadRgb48("data/testdata/test_ppm.ppm"));
    CORE_ASSERT_TRUE(ppm != NULL, "PPM Image load failed");

    std::unique_ptr<G12Buffer> g(new G12Buffer(ppm->getSize(), false));

    for (int k = 0; k < 4; k++) {
        for (int i = 0; i < ppm->h; i++) {
            for (int j = 0; j < ppm->w; j++) {
                g->element(i, j) = ppm->element(i, j)[Debayer(g.get(), 8, nullptr, k).colorFromBayerPos(i, j, false)];
            }
        }

        printf("Bayer phase %d, RMSD is: %lf\n", k, ErrorMetrics::Yrmsd(g.get(), ppm.get(), 3, 8));
    }
}
