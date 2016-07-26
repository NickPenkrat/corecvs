/**
 * \file main_testbed.cpp
 *
 * \date Sep 20, 2013
 **/

#include <string>
#include <iostream>

#include <QtGui/QtGui>
#include <QApplication>

#include "global.h"
#include "utils.h"

#include "testbedMainWindow.h"
#include "qtFileLoader.h"



using namespace std;
using namespace corecvs;

int main(int argc, char *argv[])
{
//    Q_INIT_RESOURCE(testbed);
    QTG12Loader::registerMyself();
    QTRGB24Loader::registerMyself();

    setSegVHandler();
    setStdTerminateHandler();

    printf("Starting testbed...\n");

    QApplication app(argc, argv);
    TestbedMainWindow mainWindow;

    app.exec();

    printf("Exiting testbed\n");
    return 0;

}

