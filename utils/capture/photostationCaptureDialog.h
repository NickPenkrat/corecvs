#ifndef PHOTOSTATIONCAPTUREDIALOG_H
#define PHOTOSTATIONCAPTUREDIALOG_H

#include <QSignalMapper>
#include <QDialog>

#include "imageCaptureInterface.h"
#include "capSettingsDialog.h"
#include "graphPlotDialog.h"
#include "rotaryTableControlWidget.h"
#include "abstractImageNamer.h"
#include "manipulatorCaptureDialog.h"

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
        COLUMN_SETTINGS
    };

    const static QString DEFAULT_FILENAME;

public slots:
    void setNamer(AbstractImageNamer *namer);

    void refresh();
    void capture(bool shouldAdvance = false, int positionShift = 0);
    void captureAndAdvance();
    void captureWithManipulator(int manipulatorPosition);
    void finaliseManipulatorCapture(bool advance);
    void stopCapture();
    void outputDir();

    void tableClick(int lineid, int colid);
    void previewRequest(int lineid);
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
    CapSettingsDialog       *mCapSettingsDialog = NULL;
    GraphPlotDialog          mFocusDialog;
    RotaryTableControlWidget mRotaryDialog;

    /**
     * This relates to capture process
     **/
    struct CameraDescriptor {
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

    ManipulatorCaptureDialog mManupulatorCapturer;
    bool                     mRuningManipulator;

private:
    Ui::PhotostationCaptureDialog *ui;
    AbstractImageNamer            *mNamer = NULL;

    ImageCaptureInterface*   createCameraCapture(const string &devname, bool processError = true);
};

#endif // PHOTOSTATIONCAPTUREDIALOG_H
