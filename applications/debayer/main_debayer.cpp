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

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("PGM Demosaic Tool");
    QCoreApplication::setApplicationVersion("1.0");
    QCommandLineParser parser;
    parser.setApplicationDescription("PGM Bayer to RGB PPM converter");
    parser.addHelpOption();
    parser.addPositionalArgument("file_containing_bayer.pgm", QCoreApplication::translate("main", "Raw Bayer sensor data"));

    QCommandLineOption qualOption(QStringList() << "q" << "quality",
        QCoreApplication::translate("main", "Demosaic quality"),
        QCoreApplication::translate("main", "quality"), "0");
    parser.addOption(qualOption);
    parser.process(app);

    int quality = parser.value("quality").toInt();

    PPMLoader ldr;
    MetaData* metadata = new MetaData;
    if (parser.positionalArguments().empty())
        parser.showHelp(-1);
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
