#include "core/utils/global.h"
#include <QtCore/qcoreapplication.h>
#include <QtCore/QObject>
#include <QSignalMapper>

#include "qtFileLoader.h"
#include "qtHelper.h"
#include "imageCaptureInterfaceQt.h"
#include "core/utils/log.h"
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
        delete_safe(mCaptureMapper);
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
    SYNC_PRINT(("Waiter::finilizeCapture(): called"));
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
    qApp->exit();
}

public slots:
    void onFrameReady(int camId)
    {
         SYNC_PRINT(("Waiter:onFrameReady(%d):called\n", camId));

        /* This protects the events from flooding input queue */
        static bool flushEvents = false;
        if (flushEvents) {
            SYNC_PRINT(("Event flushing\n"));
            return;
        }
        flushEvents = true;
        /* Flush all events in queue */
        QCoreApplication::processEvents();
        flushEvents = false;

        SYNC_PRINT(("Waiter:Checking data\n"));

        if (camId != mCurrentCam) {
            SYNC_PRINT(("Data form unexpected camera\n"));
            return;
        }


        if (mCurrentCam >= mCaptureInterfaces.count())
        {
            SYNC_PRINT(("Internal problem\n"));
        }

        CameraDescriptor &descr = mCaptureInterfaces[mCurrentCam];
        /* This could happen beacause of the old notifications */
        if (descr.input == NULL)
        {
            SYNC_PRINT(("Frame arrived after camera shutdown from %d", camId));
            return;
        }

        ImageCaptureInterface::FramePair pair =  descr.input->getFrameRGB24();

        /* Add frame skip */
        if (mCaptureInterfaces[mCurrentCam].toSkip > 0)
        {
            mCaptureInterfaces[mCurrentCam].toSkip--;
            SYNC_PRINT(("Skipping frame\n"));
            return;
        }

        if (pair.rgbBufferLeft() == NULL) {
            SYNC_PRINT(("Unexpected zero buffer form camera %d", camId));
            pair.freeBuffers();
            return;
        }

        mCaptureInterfaces[mCurrentCam].result = toQImage(pair.rgbBufferLeft());
        pair.freeBuffers();

        delete_safe(descr.input);

        /* Last camera */
        if (mCurrentCam == mCaptureInterfaces.size() - 1)
        {
            SYNC_PRINT(("All cameras captured\n"));
            finilizeCapture();
            return;
        }

        mCurrentCam++;
        SYNC_PRINT(("Starting %d capture \n", mCurrentCam));
        mCaptureInterfaces[mCurrentCam].input->startCapture();
    }
};
