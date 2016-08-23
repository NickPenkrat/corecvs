#include <iostream>
#include <fstream>

#include <QApplication>
#include <QtCore>

#ifdef WITH_LIBJPEG
#include "libjpegFileReader.h"
#endif
#ifdef WITH_LIBPNG
#include "libpngFileReader.h"
#endif

#include "qtFileLoader.h"


#include "calibrationCamera.h"
#include "renderer/simpleRenderer.h"
#include "mesh3d.h"
#include "meshLoader.h"
#include "objLoader.h"
#include "rgb24Buffer.h"
#include "cameraModel.h"
#include "bmpLoader.h"

extern void raytrace_scene1( void );
extern void raytrace_scene_scanner( void );
extern void raytrace_scene_calibrate( void );

extern void raytrace_scene_speedup( void );

extern void raytrace_scene_pole( void );

int main(int argc, char **argv)
{
    QApplication a(argc, argv);

#ifdef WITH_LIBJPEG
    LibjpegFileReader::registerMyself();
    SYNC_PRINT(("Libjpeg support on\n"));
#endif
#ifdef WITH_LIBPNG
    LibpngFileReader::registerMyself();
    SYNC_PRINT(("Libpng support on\n"));
#endif
    QTRGB24Loader::registerMyself();

    raytrace_scene_pole();

//    raytrace_scene1();
//    raytrace_scene_scanner();
//    raytrace_scene_calibrate();





}

