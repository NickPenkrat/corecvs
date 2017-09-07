#include <iostream>
#include "bufferFactory.h"
#include "bmpLoader.h"
#include "rgb24Buffer.h"

#ifdef WITH_LIBJPEG
#include "libjpegFileReader.h"
#endif
#ifdef WITH_LIBPNG
#include "libpngFileReader.h"
#endif

using namespace std;
using namespace corecvs;

int main(int argc, char *argv[])
{
#ifdef WITH_LIBJPEG
    LibjpegFileReader::registerMyself();
    SYNC_PRINT(("Libjpeg support on\n"));
#endif
#ifdef WITH_LIBPNG
    LibpngFileReader::registerMyself();
    SYNC_PRINT(("Libpng support on\n"));
#endif

    if (argc != 2)
       return 1;

    RGB24Buffer *out = BufferFactory::getInstance()->loadRGB24Bitmap(argv[1]);
    if (out == NULL)
    {
        SYNC_PRINT(("Unable to load image\n"));
        return 2;
    }

    BMPLoader().save("out.bmp", out);
    delete_safe(out);

    return 0;
}
