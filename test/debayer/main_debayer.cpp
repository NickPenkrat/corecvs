/*
    Bayer to PPM converter
*/
#include <iostream>

#include "ppmLoader.h"
#include "converters/debayer.h"
#include "commandLineSetter.h"
#include <time.h>
#include "converters/errorMetrics.h"

void usage()
{
    cout << "Demosaic tool"                                                                   << endl
         << "Usage:"                                                                          << endl
         << "debayer --file=bayer.pgm [--method=N] [--bpos=P] --ofile=debayer.ppm"            << endl
         << "       [--compare --target=target --method=N]"                                   << endl
         << ", where:"                                                                        << endl
         <<                                                                                      endl
         << " --file=bayer.pgm specifies input Bayer file, in this case \"bayer.pgm\""        << endl
         <<                                                                                      endl
         << " --method=N specifies demosaic method, where N:"                                 << endl
         << " \t0\tNearest Neighbor"                                                          << endl
         << " \t1\tBilinear"                                                                  << endl
         << " \t2\tAHD\t(default)"                                                            << endl
#ifdef WITH_MKL
         << " \t3\tFFT-based frequency filtering"                                             << endl
#endif
         <<                                                                                      endl
         << " --bpos=P specifies bayer position [0-3], where P=0 - RGGB (default)"            << endl
         <<                                                                                      endl
         << " --ofile=bayer.ppm specifies output color image"                                 << endl
         <<                                                                                      endl
         << " --compare indicates that debayer should work in comparison mode, ignoring all"  << endl
         << "   demosaic options and outputting comparison result"                            << endl
         <<                                                                                      endl
         << "  Comparison mode parameters:"                                                   << endl
         << "   --target=target specifies target image to compare to"                         << endl
         << "   --method=N specifies comparison method, where N:"                             << endl
         << "   \t0\tPSNR"                                                                    << endl
         << "   \t1\tRMSD"                                                                    << endl;
}

int main(int argc, const char **argv)
{
    CommandLineSetter s(argc, argv);
    bool   help     = s.getBool  ("help");
    int    method   = s.getInt   ("method", Debayer::Method::AHD);
    string filename = s.getOption("file");
    bool   toBayer  = s.getBool  ("toBayer");
    string outfile  = s.getString("ofile", "bayer.ppm");
    bool   compare  = s.getBool  ("compare");
    string target   = s.getOption("target");
    int    bpos     = s.getInt   ("bpos", -1);      // -1 - try to extract it from Bayer's meta

    if (help || argc < 2)
    {
        usage();
        return 0;
    }

    MetaData meta, metaTarget;
    if (compare)
    {
        RGB48Buffer *inRgb = PPMLoader().loadRGB(filename, &meta);
        RGB48Buffer *targetRgb = PPMLoader().loadRGB(target, &metaTarget);

        if (inRgb == nullptr)
        {
            std::cout << "Couldn't open file \"" << filename << "\"." << std::endl;
            return -1;
        }
        if (targetRgb == nullptr)
        {
            std::cout << "Couldn't open file \"" << target << "\"." << std::endl;
            return -1;
        }

        if (method == Debayer::Method::AHD || method == Debayer::Method::Nearest)
        {
            std::cout << corecvs::ErrorMetrics::psnr(inRgb, targetRgb) << endl;
        }
        else
        {
            std::cout << corecvs::ErrorMetrics::rmsd(inRgb, targetRgb) << endl;
        }
        delete_safe(inRgb);
        delete_safe(targetRgb);
        return 0;
    }


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

        Debayer d(bayer, 8, &meta, bpos);
        d.toRGB48(Debayer::Method(method), result);

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
