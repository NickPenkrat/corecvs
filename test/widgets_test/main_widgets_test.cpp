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

#ifdef INCLUDE_EXAMPLE
#include "testClass.h"
#endif

#include "qtFileLoader.h"
#include "reflectionWidget.h"
#include "axisAlignedBoxParameters.h"
#include "chessBoardAssemblerParamsBase.h"
#include "checkerboardDetectionParameters.h"
#include "chessBoardCornerDetectorParamsBase.h"
#include "iterativeReconstructionNonlinearOptimizationParamsWrapper.h"

#include "vector2d.h"

int main(int argc, char **argv)
{    
    QApplication a(argc, argv);
    Q_INIT_RESOURCE(main);


    Vector2dd dummy(0.0);
    cout << "Out:" <<  dummy.reflection.fields.size() << std::endl;

#ifdef INCLUDE_EXAMPLE
    cout << TestClass::reflection.fields.size();
#endif


#ifdef WITH_LIBJPEG
    LibjpegFileReader::registerMyself();
    SYNC_PRINT(("Libjpeg support on\n"));
#endif
#ifdef WITH_LIBPNG
    LibpngFileReader::registerMyself();
    SYNC_PRINT(("Libpng support on\n"));
#endif
    QTRGB24Loader::registerMyself();

    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();

    if (argc != 2) {
        cout << "The list of reflected objects" << endl;
        for (auto it : directory)
        {
            cout << "Class: " << it.first << endl;
        }

        return 0;
    }

    std::string className(argv[1]);

    auto it = directory.find(className);
    if (it == directory.end()) {
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
    } else {
        Reflection *widget_ref = (*it).second;
        ReflectionWidget *aabWidget = new ReflectionWidget(widget_ref);
        aabWidget->show();
    }

    a.exec();
//    raytrace_scene1();
//    raytrace_scene_scanner();
//    raytrace_scene_calibrate();

}

