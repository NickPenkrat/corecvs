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

    Debayer d(bayer, 8, &meta);

    RGB48Buffer *result = d.toRGB48(Debayer::Method(quality));

    PPMLoader().save("debayer_out.ppm", result);
    
    delete_safe(bayer);
    delete_safe(result);
    return 0;
}
