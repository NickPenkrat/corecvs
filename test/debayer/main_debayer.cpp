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
         << "Usage: "                                                                         << endl
         << "debayer --file=filename.pgm [--method=N] [--bpos=P] [--compare --target=target " << endl
         << "--method=N]"                                                                     << endl
         <<                                                                                      endl
         << " --file=filename.pgm specifies input file, in this case \"filename.pgm\""        << endl
         <<                                                                                      endl
         << " --method=N specifies demosaic method [0-2]"                                     << endl
         << " Implemented methods by N:"                                                      << endl
         << " \t0\tNearest Neighbor"                                                          << endl
         << " \t1\tBilinear"                                                                  << endl
         << " \t2\tAHD"                                                                       << endl
         <<                                                                                      endl
         << " --bpos=P specifies bayer position [0-3]"                                        << endl
         <<                                                                                      endl
         << " --compare indicates that debayer should work in comparison mode, ignoring all"  << endl
         << " other options and outputting comparison result"                                 << endl
         <<                                                                                      endl
         << " Comparison mode parameters:"                                                    << endl
         << "   --target=target specifies target image to compare to"                         << endl
         <<                                                                                      endl
         << "   --method=N specifies comparison method"                                       << endl
         << "   Implemented comparison methods by N:"                                         << endl
         << "   \t0\tPSNR"                                                                    << endl
         << "   \t1\tRMSD"                                                                    << endl;
}

int main(int argc, const char **argv)
{
    CommandLineSetter s(argc, argv);
    bool        help     = s.getBool("help");

    if (help || argc < 2)
    {
        usage();
        return 0;
    }

    int         method   = s.getInt("method", 3);
    std::string filename = s.getOption("file");
    bool        toBayer  = s.getBool("toBayer");
    std::string outfile  = s.getString("ofile", "debayer_out.ppm");
    bool        compare  = s.getBool("compare");
    std::string target   = s.getOption("target");
    int         bpos     = s.getInt("bpos", -1);

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
        if (method == 3 || method == 0)
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
