/*
    Bayer to PPM converter
*/
#include <iostream>

#include "ppmLoader.h"
#include "converters/debayer.h"
#include "commandLineSetter.h"
#include <time.h>

void usage()
{
    cout << "Demosaic tool"                                                             << endl
         << "Usage: "                                                                   << endl
         << "debayer --file=filename.pgm [--method=N] [--bpos=P]"                       << endl
         <<                                                                                endl
         << " --file=filename.pgm specifies input file, in this case \"filename.pgm\""  << endl
         <<                                                                                endl
         << " --method=N specifies demosaic method [0-2]"                               << endl
         << " Implemented methods by N:"                                                << endl
         << " \t0\tNearest Neighbor"                                                    << endl
         << " \t1\tBilinear"                                                            << endl
         << " \t2\tAHD"                                                                 << endl
         <<                                                                                endl
         << " --bpos=P specifies bayer position [0-3]"                                  << endl;
}

int main(int argc, const char **argv)
{
    if (argc == 2 && (strcmp(argv[1], "/?") == 0 || strcmp(argv[1], "/help") == 0 || strcmp(argv[1], "--help") == 0) || strcmp(argv[1], "-h") == 0)
    {
        usage();
        return 0;
    }

    CommandLineSetter s(argc, argv);

    int         quality  = s.getInt("method", 3);
    std::string filename = s.getOption("file");
    bool        toBayer  = s.getBool("toBayer");
    std::string outfile  = s.getOption("ofile");

    if (outfile == "")
    {
        outfile = "debayer_out.ppm";
    }

    int bpos = s.getInt("bpos", -1);

    MetaData meta;

    G12Buffer* bayer;
    if (!toBayer)
    {
        bayer = PPMLoader().load(filename, &meta);
        if (bayer == nullptr)
        {
            std::cout << "Couldn't open file \"" << filename << "\"." << std::endl;
            return -1;
        }

        RGB48Buffer *result = new RGB48Buffer(bayer->h, bayer->w, false);

        double time = 0;

        Debayer d(bayer, 8, &meta, bpos);
        d.fourier();
        d.toRGB48(Debayer::Method(quality), result);

        PPMLoader().save(outfile, result);
        delete_safe(result);
    }
    else
    {
        RGB48Buffer *inRgb = PPMLoader().loadRGB(filename, &meta);

        if (inRgb == nullptr)
        {
            std::cout << "Couldn't open file \"" << filename << "\"." << std::endl;
            return -1;
        }

        bayer = new G12Buffer(inRgb->h, inRgb->w, false);

        Debayer d(bayer, 8, &meta, bpos);
        d.fromRgb(inRgb);

        PPMLoader().save(outfile, bayer);

        delete_safe(inRgb);
    }


    
    delete_safe(bayer);
    return 0;
}
