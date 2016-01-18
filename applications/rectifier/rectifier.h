#ifndef RECTIFIER_APP_H
#define RECTIFIER_APP_H

#include <QtGui/QtGui>
#include <QApplication>

#include "rgb24Buffer.h"

#include "rectifyParametersControlWidget.h"
#include "pointsRectificationWidget.h"

class RectifierApp : public QApplication {
    Q_OBJECT

public:
    AutomatedRectifyControlWidget *mainWindow;
    PointsRectificationWidget *pointEditor;

    RGB24Buffer *right;
    RGB24Buffer *left;

    RectifierApp(int &argc, char **argv);

    virtual ~RectifierApp() {
        delete_safe(mainWindow);
        delete_safe(pointEditor);

        delete_safe(right);
        delete_safe(left);

    }

public slots:
    void showPointsRectificationDailog();

};

#endif // RECTIFIER_APP_H
