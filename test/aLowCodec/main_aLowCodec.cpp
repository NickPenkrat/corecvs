#include <stdio.h>

#include <QString>

#include "aLowCodec.h"
#include "qtFileLoader.h"
#include "bmpLoader.h"

#if 1
using namespace corecvs;

inline void testChan(uint16_t chan12)
{
    uint8_t  coded  = code12to8(chan12);
    uint16_t res    = decode8to12(coded);

    if (chan12 <   64) { ASSERT_TRUE_S(CORE_ABS(res - chan12) <=  0); return; }
    if (chan12 <  128) { ASSERT_TRUE_S(CORE_ABS(res - chan12) <=  1); return; }
    if (chan12 <  256) { ASSERT_TRUE_S(CORE_ABS(res - chan12) <=  2); return; }
    if (chan12 <  512) { ASSERT_TRUE_S(CORE_ABS(res - chan12) <=  4); return; }
    if (chan12 < 1024) { ASSERT_TRUE_S(CORE_ABS(res - chan12) <=  8); return; }
    if (chan12 < 2048) { ASSERT_TRUE_S(CORE_ABS(res - chan12) <= 16); return; }
    if (chan12 < 4096) { ASSERT_TRUE_S(CORE_ABS(res - chan12) <= 32); return; }

    ASSERT_FAIL("Channel value is out of range");
}

inline void testRGB(int r8, int g8, int b8)
{
    testChan(r8 << 4);
    testChan(g8 << 4);
    testChan(b8 << 4);
}

void testCodec(void)
{
    testRGB( 20,  40,  70);
    testRGB(130, 250, 255);
    testChan(350);
    testChan(600);
    testChan(1023);
    testChan(1024);
    testChan(2100);
    testChan(4095);
}
#endif

int main(int argc, char **argv)
{
    testCodec();

    QTRGB24Loader::registerMyself();
    ALowCodec codec;

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
