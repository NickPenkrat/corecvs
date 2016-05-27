#ifndef PHOTOSTATIONCAPTUREDIALOG_H
#define PHOTOSTATIONCAPTUREDIALOG_H

#include <QSignalMapper>
#include <QDialog>

#include "imageCaptureInterface.h"
#include "capSettingsDialog.h"
#include "graphPlotDialog.h"
#include "rotaryTableControlWidget.h"
#include "abstractImageNamer.h"
#include "abstractManipulatorInterface.h"
//#include "manipulatorCaptureDialog.h"

namespace Ui {
class PhotostationCaptureDialog;
}

class PhotostationCaptureDialog : public QDialog, SaveableWidget
{
    Q_OBJECT

public:
    explicit PhotostationCaptureDialog(QWidget *parent = 0);
    ~PhotostationCaptureDialog();

    enum {
        COLUMN_SYS_ID,
        COLUMN_CAM_ID,
        COLUMN_USE,
        COLUMN_PREVIEW,
        COLUMN_SETTINGS,
        COLUMN_SAVE_SETTINGS
    };

    const static QString DEFAULT_FILENAME;
    void setManipulator(AbstractManipulatorInterface *manipulator);

public slots:
    void setNamer(AbstractImageNamer *namer);

    void refresh();
    void capture(bool shouldAdvance = false, int positionShift = 0);
    void captureAndAdvance();
    void captureWithManipulator(int manipulatorPosition);
    void finalizeManipulatorCapture(bool advance);
    void stopCapture();
    void outputDir();

    void tableClick(int lineid, int colid);

    void previewRequest(int lineid);
    void previewStop();
    bool previewRunning();

    void cameraSettings(int lineid);

    void newPreviewFrame();

    void newFormatSelected(int num);

    void newCaptureFrame(int camId);
    void initateNewFrame();

    void finalizeCapture(bool isOk = true);

protected:
    virtual void showEvent    (QShowEvent * event) override;
    virtual void keyPressEvent(QKeyEvent  * event) override;

private:
    bool                     mCamsScanned = false;
    bool                     mIsCalibrationMode = false;    // autodetected flag that we are in the calibration mode


    ImageCaptureInterface   *mPreviewInterface = NULL;
    int                      mPreviewTableLine = -1;

    CapSettingsDialog       *mCapSettingsDialog = NULL;
    GraphPlotDialog          mFocusDialog;
    RotaryTableControlWidget mRotaryDialog;

    struct CameraInfo  {
        int listNumber = 0;
        std::string systemId; /**< line you need to open the camera with capture interface*/
        std::string serialId; /**< name that allows to id the physical camera */
        std::string paramsStoreId; /** < name that is used to store the parametes in config file */
    };

    vector<CameraInfo>       mCameraInfos;

    /**
     * This relates to capture process
     **/
    struct CameraDescriptor {
        CameraInfo             info;
        int                    camId = -1;
        int                    toSkip = 0;
        ImageCaptureInterface *camInterface = NULL;
        QImage                *result = NULL;

        CameraDescriptor() {}

        bool isFilled() const    { return result != NULL; }
    };

    QList<CameraDescriptor>  mCaptureInterfaces;
    QSignalMapper           *mCaptureMapper = NULL;
    bool                     mAdvanceAfterSave = false;

    AbstractManipulatorInterface  *mManipulatorCapturer = NULL;
    bool                     mRuningManipulator;

private:
    Ui::PhotostationCaptureDialog *ui;
    AbstractImageNamer            *mNamer = NULL;

    ImageCaptureInterface*   createCameraCapture(const string &devname, bool processError = true);
    void clearSignalMappings(int i);

signals:
    void captureFinished();
    void waitingNextCapturePosition();
};

#endif // PHOTOSTATIONCAPTUREDIALOG_H
