/**
* \file main_test_fftw.cpp
* \brief This is the main file for debayer color test
*/
#include <sstream>
#include <iostream>
#include "gtest/gtest.h"

#include <global.h>
#include "g12Buffer.h"
#include "ppmLoader.h"
#include "converters/debayer.h"
#include "converters/errorMetrics.h"
#include "fftw/fftwWrapper.h"

using namespace std;
using namespace corecvs;

TEST(FFTWWrapper, doublePrecisionTest)
{
    PPMLoader *ppmLoader = new PPMLoader();
    G12Buffer *ppm = ppmLoader->load("data/testdata/test_debayer.pgm");
    CORE_ASSERT_TRUE(ppm != NULL, "PPM Image load failed");
    CORE_ASSERT_TRUE(ppm->verify(), "PPM Image verification failed");
    FFTW fftw;

    fftw_complex *input = new fftw_complex[ppm->h * ppm->w];
    fftw_complex *fft = new fftw_complex[ppm->h * ppm->w];
    fftw_complex *output = new fftw_complex[ppm->h * ppm->w];

    for (int i = 0; i < ppm->h; i++)
    {
        for (int j = 0; j < ppm->w; j++)
        {
            input[i*ppm->w + j][0] = ppm->element(i, j);
            input[i*ppm->w + j][1] = 0;
        }
    }

    fftw.transform2D(ppm->w, ppm->h, input, fft, FFTW::Forward);
    fftw.transform2D(ppm->w, ppm->h, fft, output, FFTW::Backward);

    G12Buffer* fftResult = new G12Buffer(ppm->h, ppm->w, false);

    for (int i = 0; i < ppm->h; i++)
    {
        for (int j = 0; j < ppm->w; j++)
        {
            fftResult->element(i, j) = sqrt(pow(output[i * ppm->w + j][0], 2) + pow(output[i * ppm->w + j][1], 2)) / (ppm->h * ppm->w);
        }
    }
    PPMLoader().save("tmp.pgm", fftResult);
    double error = ErrorMetrics::rmsd(ppm, fftResult);
    cout << "Root-mean-square error is " << error << endl;
    CORE_ASSERT_TRUE(error < 1, "FFT Transform failed: error is too big");

    delete_safe(ppm);
    delete_safe(ppmLoader);
    delete_safe(fftResult);
}
