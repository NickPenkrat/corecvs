#ifndef PHOTOSTATIONCAPTUREDIALOG_H
#define PHOTOSTATIONCAPTUREDIALOG_H

#include <QSignalMapper>
#include <QDialog>

#include "imageCaptureInterface.h"
#include "capSettingsDialog.h"
#include "graphPlotDialog.h"
#include "rotaryTableControlWidget.h"
#include "abstractImageNamer.h"

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
    void capture(bool shouldAdvance = false);
    void captureAndAdvance();
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
    void showEvent ( QShowEvent * event );

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
        int                    camId;
        int                    toSkip;
        ImageCaptureInterface *camInterface = NULL;
        QImage                *result = NULL;

        CameraDescriptor() {}

        bool isFilled()    { return result != NULL; }
    };

    QSignalMapper           *mCaptureMapper = NULL;
    QList<CameraDescriptor>  mCaptureInterfaces;
    bool                     mAdvanceAfterSave = false;

private:
    Ui::PhotostationCaptureDialog *ui;
    AbstractImageNamer            *mNamer = NULL;
};

#endif // PHOTOSTATIONCAPTUREDIALOG_H
