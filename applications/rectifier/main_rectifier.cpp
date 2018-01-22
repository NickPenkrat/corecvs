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


#include "core/utils/global.h"
#include "core/utils/utils.h"
#include "mesh3DScene.h"
#include "qtFileLoader.h"

#include "rectifier.h"

RectifierApp::RectifierApp(int &argc, char **argv) :
    QApplication(argc, argv),
    mainWindow( new AutomatedRectifyControlWidget(NULL, true, "") ),
    pointEditor( new PointsRectificationWidget ),
    right(NULL),
    left(NULL)
{
    qDebug("RectifierApp::RectifierApp(%d, _):called", argc);
    if (argc == 3) {
        qDebug("RectifierApp::RectifierApp():opening 1 <%s>", argv[1]);
        qDebug("RectifierApp::RectifierApp():opening 2 <%s>", argv[2]);

        right = BufferFactory::getInstance()->loadRGB24Bitmap(argv[1]);
        left  = BufferFactory::getInstance()->loadRGB24Bitmap(argv[2]);
    } else {
        right = new RGB24Buffer(100, 100);
        left  = new RGB24Buffer(100, 100);
    }

    connect(mainWindow->getPointsMatchingButton(), SIGNAL(released()), this, SLOT(showPointsRectificationDailog()));

    mainWindow->show();
}


void RectifierApp::showPointsRectificationDailog()
{
    pointEditor->show();
    pointEditor->raise();
    pointEditor->setImage(left , Frames::LEFT_FRAME );
    pointEditor->setImage(right, Frames::RIGHT_FRAME);
}

int main(int argc, char *argv[])
{
    SET_HANDLERS();

    Q_INIT_RESOURCE(main);
    QTG12Loader::registerMyself();
    QTRGB24Loader::registerMyself();

    printf("Starting rectifier...\n");
    RectifierApp app(argc, argv);




    /*for (int i = 1; i < argc; i++) {
        Mesh3DScene *mesh = new Mesh3DScene();

        if (!loader.load(mesh, argv[i]))
        {
            delete_safe(mesh);
            continue;
        }
        mainWindow.addSubObject(argv[i], QSharedPointer<Scene3D>((Scene3D*)mesh));
    }*/

    app.exec();

    cout << "Exiting Host application  \n";

}

