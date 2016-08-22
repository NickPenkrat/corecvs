#include "global.h"
#include <QtCore/qcoreapplication.h>
#include <QtCore/QObject>
#include "imageCaptureInterface.h"
#include "qtFileLoader.h"
#include "qtHelper.h"
#include <QSignalMapper>
#include "log.h"
#include "g12Image.h"


class Waiter : public QObject
{
    Q_OBJECT

public:
    Waiter()
    {
        mCaptureMapper = new QSignalMapper();
        connect(mCaptureMapper, SIGNAL(mapped(int)), this, SLOT(onFrameReady(int)));
    }

    ~Waiter()
    {
        delete mCaptureMapper;
    }

    struct CameraDescriptor {
        int camId;
        int toSkip;
        ImageCaptureInterfaceQt *input;
        QImage *result;
    };

    QSignalMapper *mCaptureMapper;
    int mCurrentCam;
    QList<CameraDescriptor> mCaptureInterfaces;


void finilizeCapture()
{
    /* Save images here */
    for (int i = 0; i < mCaptureInterfaces.count(); i++)
    {
        QString name = QString("SP%1%2_%3deg.jpg")
                .arg("cam")
                .arg(mCaptureInterfaces[i].camId)
                .arg(0);
        mCaptureInterfaces[i].result->save(name);
        qDebug() << "Saving " << name;
        delete_safe(mCaptureInterfaces[i].result);
    }

    mCaptureInterfaces.clear();
}

public slots:
    void onFrameReady(int camId)
    {
         qDebug() << "Frame ready for cam " << camId;

        /* This protects the events from flooding input queue */
        static bool flushEvents = false;
        if (flushEvents) {
            return;
        }
        flushEvents = true;
        /* Flush all events in queue */
        QCoreApplication::processEvents();
        flushEvents = false;

        if (camId != mCurrentCam) {
            return;
        }

        /* Add frame skip */
        if (mCaptureInterfaces[mCurrentCam].toSkip > 0)
        {
            mCaptureInterfaces[mCurrentCam].toSkip--;
            return;
        }

        if (mCurrentCam >= mCaptureInterfaces.count())
        {
            SYNC_PRINT(("Internal problem"));
        }

        CameraDescriptor &descr = mCaptureInterfaces[mCurrentCam];
        /* This could happen beacause of the old notifications */
        if (descr.input == NULL)
        {
            L_INFO_P("Frame arrived after camera shutdown from %d", camId);
            return;
        }

        ImageCaptureInterface::FramePair pair =  descr.input->getFrameRGB24();
        if (pair.rgbBufferLeft == NULL) {
            L_ERROR_P("Unexpected zero buffer form camera %d", camId);
            pair.freeBuffers();
            return;
        }

        mCaptureInterfaces[mCurrentCam].result = toQImage(pair.rgbBufferLeft);
        pair.freeBuffers();

        delete_safe(descr.input);

        /* Last camera */
        if (mCurrentCam == mCaptureInterfaces.size() - 1)
        {
            finilizeCapture();
            return;
        }

        mCurrentCam++;
        mCaptureInterfaces[mCurrentCam].input->startCapture();
    }
};
