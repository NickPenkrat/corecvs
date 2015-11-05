/*
Bayer to PPM converter

*/

#include <queue>
#include <iostream>
#include "ppmLoader.h"
#include "converters/debayer.h"
#include "commandLineSetter.h"
//#include <qcommandlineparser.h>

using std::cout;
using std::endl;
using std::string;
using corecvs::PPMLoader;

int main(int argc, const char **argv)
{
    CommandLineSetter s(argc, argv);

    int quality = s.getInt("quality", 1);
    

    PPMLoader ldr;
    MetaData* metadata = new MetaData;
    string filename = s.getOption("file");
    G12Buffer* bayer = ldr.load(filename, metadata);
    Debayer d(bayer, 12, metadata);
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

    ldr.save("out.ppm", result);
    delete_safe(metadata);
}
