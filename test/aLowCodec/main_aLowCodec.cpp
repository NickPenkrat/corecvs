#include <stdio.h>

#include <QString>

#include "aLowCodec.h"
#include "qtFileLoader.h"
#include "bmpLoader.h"

int main(int argc, char **argv)
{
    QTRGB24Loader::registerMyself();

    if (argc < 2) {
        printf("Use test_aLowCodec [--code|--decode] in_filename.{bmp|jpg} <out_filename_{.bmp|_85.jpg|.jpg}>\n");
        return 1;
    }

    bool   code = false;
    bool decode = false;
    string in("../../../../data/testdata/distortion/SPA0_360deg.jpg");
    string oPath;

    if (argc >= 2) {
        int idxIn = 1;
        if (QString("--code") == argv[1])
        {
            code = true;
            idxIn = 2;
        }
        else if (QString("--decode") == argv[1])
        {
            decode = true;
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

    RGB24Buffer *out = image;
    if (code)
    {
        out = ALowCodec::code(image);
    }
    else if (decode)
    {
        out = ALowCodec::decode(image);
    }
    if (out == NULL)
    {
        SYNC_PRINT(("Failed to %s the image <%s>.\n", code ? "code" : (decode ? "decode" : "convert"), in.c_str()));
        return -2;
    }

    if (oPath.empty())
    {
        BMPLoader().save("result.bmp", out);
        QTFileLoader().save("result100.jpg", out, 100);
        QTFileLoader().save("result85.jpg", out, 85);
    }
    else
    {
        QString oPathLwr = QString(oPath.c_str()).toLower();
        if (oPathLwr.indexOf(".bmp") > 0)
            BMPLoader().save(oPath, out);
        else if (oPathLwr.indexOf("_85.jpg") > 0)
            QTFileLoader().save(oPath, out, 85);
        else if (oPathLwr.indexOf("_50.jpg") > 0)
            QTFileLoader().save(oPath, out, 50);
        else
            QTFileLoader().save(oPath, out, 100);
    }
    if (out != image) {
        delete_safe(out);
    }
    delete_safe(image);
    return 0;
}
