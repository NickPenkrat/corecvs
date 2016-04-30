#include "mainQScriptWindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainQScriptWindow w;
    w.show();

    return a.exec();
}
