/*
    CR2 Bayer extractor console tool.
    Internally uses LibRaw.

*/

#include <queue>
#include <iostream>

#include "cr2reader.h"
#include "core/fileformats/ppmLoader.h"
#include "core/reflection/commandLineSetter.h"

using std::cout;
using std::endl;
using std::string;

void usage(char *argv[])
{
    cout << "Raw CR2 to PPM converter. Usage:" << endl
        << argv[0] << " [-rp] [-b number] file1.cr2 file2.cr2" << endl
        << "-r\tforce 8-bit PPM format for more than 8-bit data" << endl
        << "-b N\tuse N-bit PPM format, where N is from 1 to 16" << endl
        << "-p\toutput dcraw-proccessed image instead of Bayer sensor data" << endl;
}

int main(int argc, const char **argv)
{
    CR2Reader rdr;
    CommandLineSetter s(argc, argv);
    int bits = s.getInt("b", 12);
    rdr.setBPP(bits);
    string file = s.getString("file", "");

    int result = 0;
    if ((result = rdr.open(file)) == 0)
    {
        G12Buffer* bayer = rdr.getBayer(true);
        MetaData* meta = rdr.getMetadata();

        PPMLoader().save((string(file) + ".pgm").c_str(), bayer, meta);
        delete_safe(bayer);
        delete_safe(meta);
    }
    else
        cout << "Could not open file " << file << endl;
    return result;
}
