/**
 * \file main_qt_recorder.cpp
 * \brief Entry point for the recorder application
 *
 * \date Sep 17, 2010
 * \author Sergey Levi
 */

#include <string>
#include <iostream>

#include <QtGui/QtGui>
#include <QApplication>


#include "global.h"
#include "utils.h"
#include "advancedImageWidget.h"
#include "qtFileLoader.h"
#include "g12Image.h"
#include "ppmLoader.h"

#include "imageViewMainWindow.h"

using namespace std;

int main(int argc, char *argv[])
{
    setSegVHandler();
    setStdTerminateHandler();

    Q_INIT_RESOURCE(main);

    printf("Starting cloudView...\n");
    QApplication app(argc, argv);
    ImageViewMainWindow mainWindow;


    QTG12Loader::registerMyself();
    QTRGB24Loader::registerMyself();

    if (argc != 0) {
        std::string str = argv[1];
        qDebug("Main: %s", str.c_str());

        RGB48Buffer *buffer = PPMLoader().rgb48BufferCreateFromPPM(str);
        if (buffer == NULL)
        {
            qDebug("Can't' open file: %s", str.c_str());
        } else {
            mainWindow.input = buffer;
        }
    }


    mainWindow.show();
    app.exec();

    cout << "Exiting ImageView application  \n";

}

