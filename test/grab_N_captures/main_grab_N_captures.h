#include "global.h"
#include <stdio.h>
#include <QtCore/QObject>
#include <QtCore/QCoreApplication>
#include <QtCore/QThread>
#include <QtCore/QMutex>
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
#include "qtFileLoader.h"


class Waiter : public QObject
{
    Q_OBJECT
public:
    Waiter(){}
    ImageCaptureInterface *input;
    int *frameToSkip;

public slots:
    void onFrameReady()
    {
        (*frameToSkip)--;
        if(*frameToSkip != 1)
        {
            return;
        }

        SYNC_PRINT(("Hello \n"));
        QString captureName;
        QString leftDeviceName;
        QString rightDeviceName;

        ImageCaptureInterface::FramePair pair = input->isRgb ? input->getFrameRGB24() : input->getFrame();

        input->getCaptureName(captureName);
        input->getDeviceName(0,leftDeviceName);
        input->getDeviceName(1,rightDeviceName);

        leftDeviceName = leftDeviceName.replace("/","_");
        rightDeviceName = rightDeviceName.replace("/","_");

        if (pair.bufferLeft != NULL)
            saveToFile(new RGB24Buffer(pair.bufferLeft), captureName + "_" + leftDeviceName);

        if (pair.bufferRight != NULL)
            saveToFile(new RGB24Buffer(pair.bufferRight), captureName + "_" + rightDeviceName);

        if (pair.rgbBufferLeft != NULL)
            saveToFile(pair.rgbBufferLeft, captureName + "_" + leftDeviceName + "_RGB");

        if (pair.rgbBufferRight != NULL)
            saveToFile(pair.rgbBufferRight, captureName + "_" + rightDeviceName + "_RGB");

        delete_safe(pair.bufferRight);
        delete_safe(pair.bufferLeft);
        delete_safe(pair.rgbBufferRight);
        delete_safe(pair.rgbBufferLeft);
    }

private:
    void saveToFile(RGB24Buffer *resultRGB, const QString &suffix){
        ASSERT_TRUE(resultRGB != NULL, "Null buffer could not be saved");
        QString filename = "snapshot_" + suffix + ".jpg";
        QTFileLoader().save(filename.toStdString(), resultRGB);
        printf("%s saved!\n", filename.toStdString().c_str());
    }
};
