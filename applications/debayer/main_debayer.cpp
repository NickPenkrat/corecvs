/*
Bayer to PPM converter

*/

#include <queue>
#include <iostream>
#include "ppmLoader.h"
#include "converters/debayer.h"

#include <qcommandlineparser.h>

using std::cout;
using std::endl;
using std::string;
using corecvs::PPMLoader;

void usage(char *argv[])
{
    cout << "PGM Bayer to RGB PPM converter. Usage:" << endl
        << argv[0] << " [-r] [-b number] [-q number] file1.pgm file2.pgm" << endl
        << "-q N\tdemosaic quality:" << endl
        << "\t0:\t nearest neighbour" << endl
        << "\t1:\t bilinear" << endl
        << "-b N\tuse N-bit PPM format, where N is from 1 to 16" << endl
        << "-r\tforce 8-bit PPM format for more than 8-bit data; shortcut for -b 8" << endl;
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("my-copy-program");
    QCoreApplication::setApplicationVersion("1.0");    QCommandLineParser parser;
    parser.setApplicationDescription("PGM Bayer to RGB PPM converter");
    parser.addHelpOption();
    parser.addPositionalArgument("bayer", QCoreApplication::translate("main", "Raw Bayer sensor data"));

    QCommandLineOption qualOption(QStringList() << "q" << "quality",
        QCoreApplication::translate("main", "Demosaic quality"),
        QCoreApplication::translate("main", "quality"), "0");
    parser.addOption(qualOption);

    parser.process(app);

    int quality = parser.value("quality").toInt();

    PPMLoader ldr;
    MetaData* metadata = new MetaData;
    string filename = parser.positionalArguments()[0].toStdString();
    G12Buffer* bayer = ldr.load(filename, metadata);
    Debayer d(bayer, 12, metadata);
    RGB48Buffer *result = nullptr;

    switch (quality)
    {
    case 0:
    default:
        result = d.toRGB48(Debayer::Nearest);
        break;
    case 1:
        result = d.toRGB48(Debayer::Bilinear);
        break;
    }

    ldr.save("out.ppm", result);
}
