/*
    Bayer to PPM converter
*/
#include <iostream>
#include <time.h>
#include "ppmLoader.h"
#include "converters/debayer.h"
#include "commandLineSetter.h"
#include <math.h>

float InvSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x;            // store floating-point bits in integer
    i = 0x5f375a86 - (i >> 1);    // initial guess for Newton's method
    x = *(float*)&i;              // convert new bits into float
    x = x*(1.5f - xhalf*x*x);     // One round of Newton's method
    return x;
}

int main(int argc, const char **argv)
{
    CommandLineSetter s(argc, argv);

    int         quality = s.getInt("quality", 1);
    std::string filename = s.getOption("file");

    MetaData meta;
    G12Buffer* bayer = PPMLoader().load(filename, &meta);
    if (bayer == NULL)
    {
        std::cout << "Couldn't open file " << filename << std::endl;
        return -1;
    }

    Debayer d(bayer, 8, &meta);

    RGB48Buffer *result = nullptr;
    cout << InvSqrt(4.0f) << endl;
    clock_t start, end;
    start = clock();
    float tmp = 0;
    for (int i = 0; i < 1000000; i++)
    {
        tmp = InvSqrt(i);
        //tmp = 1.f / sqrtf(i);
    }
    end = clock();
    double msecs = ((double)(end - start)) * 1000 / CLOCKS_PER_SEC;
    cout << msecs;
    d.fourier();
    //result = d.toRGB48(Debayer::Method(quality));

   // PPMLoader().save("out.ppm", result);

    delete_safe(bayer);
    delete_safe(result);
}
