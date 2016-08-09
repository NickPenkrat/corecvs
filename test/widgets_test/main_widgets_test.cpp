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
#include "reflectionWidget.h"
#include "axisAlignedBoxParameters.h"
#include "chessBoardAssemblerParamsBase.h"
#include "checkerboardDetectionParameters.h"
#include "chessBoardCornerDetectorParamsBase.h"
#include "iterativeReconstructionNonlinearOptimizationParamsWrapper.h"

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

    {
        Reflection *widget_ref = &AxisAlignedBoxParameters::reflection;
        ReflectionWidget *aabWidget = new ReflectionWidget(widget_ref);
        aabWidget->show();
    }

    {
        Reflection *widget_ref = &RgbColorParameters::reflection;
        ReflectionWidget *aabWidget = new ReflectionWidget(widget_ref);
        aabWidget->show();
    }

    {
        Reflection *widget_ref = &ChessBoardAssemblerParamsBase::reflection;
        ReflectionWidget *aabWidget = new ReflectionWidget(widget_ref);
        aabWidget->show();
    }

    {
        Reflection *widget_ref = &CheckerboardDetectionParameters::reflection;
        ReflectionWidget *aabWidget = new ReflectionWidget(widget_ref);
        aabWidget->show();
    }

    {
        Reflection *widget_ref = &ChessBoardCornerDetectorParamsBase::reflection;
        ReflectionWidget *aabWidget = new ReflectionWidget(widget_ref);
        aabWidget->show();
    }

    {
        Reflection *widget_ref = &IterativeReconstructionNonlinearOptimizationParamsWrapper::reflection;
        ReflectionWidget *aabWidget = new ReflectionWidget(widget_ref);
        aabWidget->show();
    }

    a.exec();
//    raytrace_scene1();
//    raytrace_scene_scanner();
//    raytrace_scene_calibrate();

}

