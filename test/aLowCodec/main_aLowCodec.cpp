#include <stdio.h>

#include <QString>

#include "aLowCodec.h"
#include "qtFileLoader.h"
#include "bmpLoader.h"

#if 0
using namespace corecvs;

inline bool testRGB(int r, int g, int b) {
    uint8_t rc = code12to8((uint16_t)r << 4);
    uint8_t gc = code12to8((uint16_t)g << 4);
    uint8_t bc = code12to8((uint16_t)b << 4);

    uint8_t rd = decode8to12(rc) >> 4;
    uint8_t gd = decode8to12(gc) >> 4;
    uint8_t bd = decode8to12(bc) >> 4;

    ASSERT_TRUE_P(CORE_ABS(rd - r) <= 32);
    ASSERT_TRUE_P(CORE_ABS(gd - g) <= 32);
    ASSERT_TRUE_P(CORE_ABS(bd - b) <= 32);
}

void testCodec(void)
{
    testRGB( 20,  40,  70);
    testRGB(130, 250, 250);
}
#endif

int main (int argc, char **argv)
{
    //testCodec();

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
