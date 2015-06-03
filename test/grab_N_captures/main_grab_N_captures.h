#include "global.h"
#include <stdio.h>
#include <QtCore/QObject>
#include <QtCore/QCoreApplication>
#include <QtCore/QThread>
#include "QTimer"
#ifdef WIN32
# include <windows.h>
# define tSleep  Sleep
#else
# include <unistd.h>
# include <stdlib.h>
# define tSleep _sleep
#endif
#include "imageCaptureInterface.h"
#include "bmpLoader.h"
#include "QTFileLoader.h"


class Waiter : public QObject
{
    Q_OBJECT
public:
    Waiter(){}
    ImageCaptureInterface *input;

public slots:
    void onFrameReady()
    {
        SYNC_PRINT(("Hello \n"));
        RGB24Buffer *resultRGB = NULL;
        QString captureName;
        QString leftDeviceName;
        QString rightDeviceName;

        ImageCaptureInterface::FramePair pair = input->isRgb ? input->getFrameRGB24() : input->getFrame();

        input->getCaptureName(captureName);
        input->getDeviceName(0,leftDeviceName);
        input->getDeviceName(1,rightDeviceName);

        if (pair.bufferLeft != NULL)
        {
            resultRGB = new RGB24Buffer(pair.bufferLeft);
            if(resultRGB != NULL)
            {
                QString filename = "snapshot_";
                filename += captureName;
                filename += "_";
                filename += leftDeviceName;
                filename += ".jpg";
                QTFileLoader().save(filename.toStdString(), resultRGB);
                printf("%s saved!\n", filename.toStdString().c_str());
                resultRGB = NULL;
            }
        }

        if (pair.bufferRight != NULL)
        {
            resultRGB = new RGB24Buffer(pair.bufferRight);
            if(resultRGB != NULL)
            {
                QString filename = "snapshot_";
                filename += captureName;
                filename += "_";
                filename += rightDeviceName;
                filename += ".jpg";
                QTFileLoader().save(filename.toStdString(), resultRGB);
                printf("%s saved!\n", filename.toStdString().c_str());
                resultRGB = NULL;
            }
        }

        if (pair.rgbBufferLeft != NULL)
        {
            resultRGB = pair.rgbBufferLeft;
            if(resultRGB != NULL)
            {
                QString filename = "snapshot_";
                filename += captureName;
                filename += "_";
                filename += leftDeviceName;
                filename += "_RGB.jpg";
                QTFileLoader().save(filename.toStdString(), resultRGB);
                printf("%s saved!\n", filename.toStdString().c_str());
                resultRGB = NULL;
            }
        }

        if (pair.rgbBufferRight != NULL)
        {
            resultRGB = pair.rgbBufferRight;
            if(resultRGB != NULL)
            {
                QString filename = "snapshot_";
                filename += captureName;
                filename += "_";
                filename += rightDeviceName;
                filename += "_RGB.jpg";
                QTFileLoader().save(filename.toStdString(), resultRGB);
                printf("%s saved!\n", filename.toStdString().c_str());
                resultRGB = NULL;
            }
        }

        delete_safe(pair.bufferRight);
        delete_safe(pair.bufferLeft);
        delete_safe(pair.rgbBufferRight);
        delete_safe(pair.rgbBufferLeft);

//        QCoreApplication::quit();
    }
};
