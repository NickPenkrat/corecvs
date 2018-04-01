#include "core/buffers/rgb24/abstractPainter.h"
#include "core/buffers/rgb24/rgb24Buffer.h"

#ifdef WITH_AVCODEC
#include "avEncoder.h"
#endif

using namespace corecvs;

int main(int argc, char **argv)
{
#ifdef WITH_AVCODEC
    AVEncoder encoder;
    RGB24Buffer buffer(100,100);

    AVEncoder::printCaps();

    encoder.startEncoding("out.avi", buffer.h, buffer.w);
    for (int i = 0; i < 250; i++)
    {
        buffer.checkerBoard(i % 99, RGBColor::Yellow(), RGBColor::Brown());
        AbstractPainter<RGB24Buffer> p(&buffer);
        p.drawFormat(0,0, RGBColor::Navy(), 2, "%d", i);
        encoder.addFrame(&buffer);
    }
    encoder.endEncoding();
#endif
    SYNC_PRINT(("Done."));
}

