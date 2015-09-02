#include <stdio.h>

#include <QString>

#include "aLowCodec.h"
#include "qtFileLoader.h"
#include "bmpLoader.h"

#if 1
using namespace corecvs;

inline bool checkChanError(int chan, int res)
{
    if (chan <   64) ASSERT_TRUE_S(CORE_ABS(res - chan) <=  0);
    if (chan <  128) ASSERT_TRUE_S(CORE_ABS(res - chan) <=  1);
    if (chan <  256) ASSERT_TRUE_S(CORE_ABS(res - chan) <=  2);
    if (chan <  512) ASSERT_TRUE_S(CORE_ABS(res - chan) <=  4);
    if (chan < 1024) ASSERT_TRUE_S(CORE_ABS(res - chan) <=  8);
    if (chan < 2048) ASSERT_TRUE_S(CORE_ABS(res - chan) <= 16);
    if (chan < 4096) ASSERT_TRUE_S(CORE_ABS(res - chan) <= 32);
    ASSERT_FAIL("Channel value is out of range");
}

inline void testRGB(int r8, int g8, int b8)
{
    uint8_t rc = code12to8((uint16_t)r8 << 4);
    uint8_t gc = code12to8((uint16_t)g8 << 4);
    uint8_t bc = code12to8((uint16_t)b8 << 4);

    uint8_t rd = decode8to12(rc) >> 4;
    uint8_t gd = decode8to12(gc) >> 4;
    uint8_t bd = decode8to12(bc) >> 4;

    checkChanError(r8, rd);
    checkChanError(g8, gd);
    checkChanError(b8, bd);
}

void testCodec(void)
{
    testRGB( 20,  40,  70);
    testRGB(130, 250, 255);
}
#endif

int main (int argc, char **argv)
{
    //testCodec();

    QTRGB24Loader::registerMyself();
    ALowCodec codec;

    if (argc < 2) {
        printf("Use test_aLowCodec [--decode|--code] <in_filename> <out_filename>");
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
        out = codec.code(image);
    }
    else if (decode)
    {
        out = codec.decode(image);
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
        QString oPathUp = QString(oPath.c_str()).toLower();
        if (oPathUp.indexOf(".bmp") > 0)
            BMPLoader().save(oPath, out);
        else if (oPathUp.indexOf("_85.jpg") > 0)
            QTFileLoader().save(oPath, out, 85);
        else
            QTFileLoader().save(oPath, out, 100);
    }
    if (out != image) {
        delete_safe(out);
    }
    delete_safe(image);
    return 0;
}
