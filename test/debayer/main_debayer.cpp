/*
    Bayer to PPM converter
*/
#include <iostream>

#include "ppmLoader.h"
#include "converters/debayer.h"
#include "commandLineSetter.h"

int main(int argc, const char **argv)
{
    CommandLineSetter s(argc, argv);

    int         quality  = s.getInt("quality", 1);
    std::string filename = s.getOption("file");

    MetaData meta;
    G12Buffer* bayer = PPMLoader().load(filename, &meta);
    if (bayer == NULL)
    {
        std::cout << "Couldn't open file " << filename << std::endl;
        return -1;
    }

    Debayer d(bayer, 12, &meta);

    RGB48Buffer *result = nullptr;

    switch (quality)
    {
    case 0:
        result = d.toRGB48(Debayer::Nearest);
        break;
    case 1:
    default:
        result = d.toRGB48(Debayer::Bilinear);
        break;
    /*case 7:
        result = d.toRGB48(Debayer::Improved);
        break;*/
    }

    PPMLoader().save("out.ppm", result);
}
