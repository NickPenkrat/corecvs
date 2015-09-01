#include <stdio.h>

#include <QString>

#include "aLowCodec.h"
#include "qtFileLoader.h"
#include "bmpLoader.h"

int main (int argc, char **argv)
{
    QTRGB24Loader::registerMyself();
    ALowCodec codec;

    if (argc < 2) {
        printf("Usage test_aLowCodec [--decode] <in_filename> <out_filename>");
    }

    bool code = true;
    string in("../../../../data/testdata/distortion/SPA0_360deg.jpg");
    string oPath;

    if (argc >= 2) {
        int idxIn = 1;
        if (QString("--decode") == argv[1])
        {
            code = false;
            idxIn = 2;
        }
        if (argc >= 1 + idxIn) {
            in = argv[idxIn];

            if (argc > 1 + idxIn) {
                oPath = argv[idxIn + 1];
            }
        }
    }

    RGB24Buffer *image = BufferFactory::getInstance()->loadRGB24Bitmap(in);
    if (image == NULL)
    {
        SYNC_PRINT(("Failed to load <%s>.\n", in.c_str()));
        return -1;
    }

    RGB24Buffer *out;
    if (code)
    {
        out = codec.code(image);
    }
    else
    {
        out = codec.decode(image);
    }
    if (out == NULL)
    {
        SYNC_PRINT(("Failed to %s the image <%s>.\n", code ? "code" : "decode", in.c_str()));
        return -2;
    }

    if (oPath.empty())
    {
        BMPLoader().save("result.bmp", out);
        QTFileLoader().save("result100.jpg", out, 100);
        QTFileLoader().save("result85.jpg", out, 85);
    }
    else {
        if (oPath.find(".bmp"))
            BMPLoader().save(oPath, out);
        else
            QTFileLoader().save(oPath, out, 100);
    }
    delete_safe(out);
    delete_safe(image);
    return 0;
}
