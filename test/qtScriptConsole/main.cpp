#include "scriptWindow.h"
#include <QApplication>
#include "coreToScript.h"

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    ScriptWindow w;

#if 0
    ReflectionToScript axisScriptObject(&AxisAlignedBoxParameters::reflection);
    axisScriptObje
    w.mEngine
#endif

    w.show();

    return a.exec();
}
