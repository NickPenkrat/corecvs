#include "global.h"
#include <QtCore/QObject>
//#include <QtCore/QThread>
//#include <QtCore/QMutex>
#include "imageCaptureInterface.h"
//#include "bmpLoader.h"
#include "qtFileLoader.h"


class Waiter : public QObject
{
    Q_OBJECT

public:
    Waiter() {}

    ImageCaptureInterface *input;
    int *frameToSkip;

public slots:
    void onFrameReady()
    {
        (*frameToSkip)--;
        if (*frameToSkip != 1)
            return;

        SYNC_PRINT(("Hello from onFrameReady()\n"));
        QString captureName;
        QString leftDeviceName;
        QString rightDeviceName;

        ImageCaptureInterface::FramePair pair = input->isRgb ? input->getFrameRGB24() : input->getFrame();

        input->getCaptureName(captureName);
        input->getDeviceName(0, leftDeviceName);
        input->getDeviceName(1, rightDeviceName);

        leftDeviceName  = leftDeviceName .replace("/", "_");
        rightDeviceName = rightDeviceName.replace("/", "_");

        if (pair.bufferLeft != NULL) {
            RGB24Buffer* res = new RGB24Buffer(pair.bufferLeft);
            saveToFile(res, captureName + "_" + leftDeviceName);
            delete_safe(res);
        }

        if (pair.bufferRight != NULL) {
            RGB24Buffer* res = new RGB24Buffer(pair.bufferRight);
            saveToFile(res, captureName + "_" + rightDeviceName);
            delete_safe(res);
        }

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
    void saveToFile(RGB24Buffer *resultRGB, const QString &suffix)
    {
        ASSERT_TRUE(resultRGB != NULL, "Null buffer could not be saved");
        QString filename = "snapshot_" + suffix + ".jpg";
        QTFileLoader().save(filename.toStdString(), resultRGB);
        printf("File <%s> saved!\n", filename.toStdString().c_str());
    }
};
