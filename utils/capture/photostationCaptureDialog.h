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
        COLUMN_PS_ID,
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
    bool                   mCamsScanned;

    ImageCaptureInterface *mPreviewInterface;
    CapSettingsDialog     *mCapSettingsDialog;
    GraphPlotDialog        focusDialog;
    RotaryTableControlWidget rotaryDialog;

    /**
     * This relates to capture process
     **/
    struct CameraDescriptor {
        int camId;
        int toSkip;
        ImageCaptureInterface *camInterface;
        QImage *result;

        CameraDescriptor()  :
            camInterface(NULL),
            result(NULL)
        {}

        bool isFilled() {
            return (result != NULL);
        }
    };

    QSignalMapper           *mCaptureMapper;
    // int mCurrentCam;
    QList<CameraDescriptor>  mCaptureInterfaces;
    bool                     mAdvanceAfterSave;

private:
    Ui::PhotostationCaptureDialog *ui;
    AbstractImageNamer *mNamer;
};

#endif // PHOTOSTATIONCAPTUREDIALOG_H
