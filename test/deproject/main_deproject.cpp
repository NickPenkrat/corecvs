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

using namespace std;
using namespace corecvs;

void usage()
{
    cout << "Deproject tool"                                                                    << endl
         << "Usage:"                                                                           << endl
         << "deproject --file=input.[bmp|jpg] --camera=cam.json] [--camtxt=cam.txt]" << endl;

}

int main(int argc, char **argv)
{
    QApplication a(argc, argv);

    CommandLineSetter s(argc, (const char**)argv);
    bool   help       = s.getBool  ("help");
    string filename   = s.getOption("file");
    string outname    = s.getOption("o");
    string camerajson     = s.getOption("camera");
    string camtxt     = s.getOption("camtxt");

    if (outname.empty()) outname = "out.bmp";

    if (help || argc < 2)
    {
        usage();
        return 0;
    }



    QTG12Loader::registerMyself();
    QTRGB24Loader::registerMyself();



    RGB24Buffer *in = BufferFactory::getInstance()->loadRGB24Bitmap(filename);
    if (in == NULL)
    {
        SYNC_PRINT(("Unable to load file"));
        return 1;
    }

    CameraModel model;
    if (!camtxt.empty())
    {
        model.intrinsics.reset(ILFormat::loadIntrisics(camtxt));
    }

    RGB24Buffer *out = new RGB24Buffer(in->getSize());
    PinholeCameraIntrinsics pinhole(Vector2dd(in->w, in->h), degToRad(120));

    int count = 0;
    parallelable_for(0, out->h, [&](const BlockedRange<int>& r)
    {
        for(int i = r.begin(); i < r.end(); i++)
        {
            for (int j = 0; j < out->w;  j++)
            {
                Vector3dd ray = pinhole.reverse(Vector2dd(j,i));
                ray.normalise();
                Vector2dd source = model.intrinsics->project(ray);
                if (in->isValidCoordBl(source)) {
                    out->element(i,j) = in->elementBl(source);
                } else {
                    out->element(i,j) = RGBColor::Gray();
                }

            }
            count++;
            if ((count % 50) == 0)
            {
                SYNC_PRINT(("...%.2lf%%", (double)count / out->h * 100));
            }
        }
    });
    SYNC_PRINT(("\n"));


    BufferFactory::getInstance()->saveRGB24Bitmap(out, outname);


    delete_safe(in);
    delete_safe(out);
    return 0;
}
