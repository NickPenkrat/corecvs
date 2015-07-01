#include <stdio.h>
#include <QApplication>
//#include <unistd.h>

#include <QImage>
//#include "graphPlotParametersControlWidget.h"
#include "rgbColorParametersControlWidget.h"
#include "vectorWidget.h"


int main (int argc, char **argv)
{
    QApplication a(argc, argv);
    VectorWidget w;
    w.mFabric = new ParametersControlWidgetBaseFabricImpl<RgbColorParametersControlWidget>;

    w.show();

    return a.exec();
}
