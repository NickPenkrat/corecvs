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

    int         quality  = s.getInt("method", 1);
    std::string filename = s.getOption("file");

    MetaData meta;
    
    G12Buffer* bayer = PPMLoader().load(filename, &meta);

    if (bayer == NULL)
    {
        std::cout << "Couldn't open file \"" << filename << "\"." << std::endl;
        return -1;
    }

    
    int bpos = meta["b_pos"].empty() ? 0 : meta["b_pos"][0];
    bpos = s.getInt("bpos", bpos);

    Debayer d(bayer, 8, bpos, &meta);

    RGB48Buffer *result = d.toRGB48(Debayer::Method(quality));

    double time = 0;

    //
    for (int i = 0; i < 10; i++)
    {
        double start = clock();

        result = d.toRGB48(Debayer::Method(3));

        double end = clock();
        double curtime = double(end - start) / CLOCKS_PER_SEC;
        time += curtime;
        cout << curtime << endl;
    }
    cout << endl << time / 10 << endl;
    getchar();
    //

    PPMLoader().save("debayer_out.ppm", result);
    
    delete_safe(bayer);
    delete_safe(result);
    return 0;
}
