#include "global.h"
#include <stdio.h>
#include <QtCore/QObject>
#include <QtCore/QCoreApplication>
#include <QtCore/QThread>
#ifdef WIN32
# include <windows.h>
# define tSleep  Sleep
#else
# include <unistd.h>
# include <stdlib.h>
# define tSleep _sleep
#endif
#include "imageCaptureInterface.h"
//#include "V4L2Capture.h"
#include "bmpLoader.h"
#include "QTFileLoader.h"


class Waiter : public QObject
{
    Q_OBJECT
public:
    Waiter(){}//:QObject(NULL)
//    {
//        input = NULL;
//    }
    ImageCaptureInterface *input;

public slots:
    void onFrameReady()
    {
        SYNC_PRINT(("Hello"));
        G12Buffer *result = NULL;
        RGB24Buffer *resultRGB = NULL;
//        QString name;

        printf("isRGB %i ____$$$$$$$$$$$$$$$_____\n", input->isRgb);

        ImageCaptureInterface::FramePair pair = input->isRgb ? input->getFrameRGB24() : input->getFrame();

//        input->getCaptureName(name);

        if (pair.bufferLeft != NULL)
        {
            result = pair.bufferLeft;
            if(result != NULL)
            {
            BMPLoader().save("snapshotL1.bmp", result);
//                QString filename = "snapshot";
////                filename += name;
//                filename += "l1.bmp";
//                BMPLoader().save(filename.toStdString(), result);
//                resultRGB = new RGB24Buffer(result);
//                QTFileLoader().save("1.jpg", resultRGB);
                result = NULL;
            }
        }

        if (pair.bufferRight != NULL)
        {
            result = pair.bufferRight;
            if(result != NULL)
            {
                BMPLoader().save("snapshotR1.bmp", result);

//                resultRGB = new RGB24Buffer(result);

//                QString filename = "snapshot";
//                filename += name;
//                filename += "l1.jpg";

//                QTFileLoader().save(filename.toStdString(), resultRGB);

//                resultRGB = NULL;
                result = NULL;
            }
        }

        if (pair.rgbBufferLeft != NULL)
        {
            resultRGB = pair.rgbBufferLeft;
            if(resultRGB != NULL)
            {
                BMPLoader().save("snapshotRL1.bmp", resultRGB);
                resultRGB = NULL;
            }
        }

        if (pair.rgbBufferRight != NULL)
        {
            resultRGB = pair.rgbBufferRight;
            if(resultRGB != NULL)
            {
                BMPLoader().save("snapshotRR1.bmp", resultRGB);
                resultRGB = NULL;
            }
        }

        delete_safe(pair.bufferRight);
        delete_safe(pair.bufferLeft);
        delete_safe(pair.rgbBufferRight);
        delete_safe(pair.rgbBufferLeft);

        delete_safe(result);

        QCoreApplication::quit();
    }
};
