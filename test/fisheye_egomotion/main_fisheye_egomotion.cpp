/*
    Bayer to PPM converter
 */

#include <QApplication>
#include <iostream>
#include <time.h>

#include "core/buffers/bufferFactory.h"
#include "core/cameracalibration/cameraModel.h"
#include "core/buffers/converters/errorMetrics.h"
#include "core/reflection/commandLineSetter.h"
#include "core/utils/utils.h"
#include "qtFileLoader.h"
#include "core/cameracalibration/ilFormat.h"


#include "moc-flow/openCVMovingObjectFlow.h"

using namespace std;
using namespace corecvs;


int main(int argc, char **argv)
{

#ifdef WITH_LIBJPEG
    LibjpegFileReader::registerMyself();
    SYNC_PRINT(("Libjpeg support on\n"));
#endif
#ifdef WITH_LIBPNG
    LibpngFileReader::registerMyself();
    SYNC_PRINT(("Libpng support on\n"));
#endif
    QTG12Loader::registerMyself();
    QTRGB24Loader::registerMyself();



    RGB24Buffer *input1 = BufferFactory::getInstance()->loadRGB24Bitmap("in0.jpg");
    RGB24Buffer *input2 = BufferFactory::getInstance()->loadRGB24Bitmap("in1.jpg");
    RGB24Buffer *input3 = BufferFactory::getInstance()->loadRGB24Bitmap("in2.jpg");






}
