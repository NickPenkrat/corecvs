#include "photostationCaptureDialog.h"

#include <QMessageBox>
#include <QFileDialog>

#include "log.h"
#include "focusEstimator.h"
#include "preciseTimer.h"

#include "g12Image.h"
#include "qtHelper.h"

#include "ui_photostationCaptureDialog.h"
#include "imageCaptureInterface.h"

/* Temporary solution. This need to be hidden inside image capture interface */
#if 0
#ifdef Q_OS_WIN
# ifdef WITH_DIRECTSHOW
#  include "directShow.h"
#  include "directShowCapture.h"
#  define CAPTURE_INTERFACE DirectShowCaptureInterface
# endif
#else
//# include "V4L2Capture.h"
//# define CAPTURE_INTERFACE V4L2CaptureInterface

# include "uEyeCapture.h"
# define CAPTURE_INTERFACE UEyeCaptureInterface
#endif
#endif

const QString PhotostationCaptureDialog::DEFAULT_FILENAME = "capture.ini";

using corecvs::FocusEstimator;

PhotostationCaptureDialog::PhotostationCaptureDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::PhotostationCaptureDialog)
{
    setWindowFlags(Qt::Window);
    ui->setupUi(this);
    ui->previewWidget->setFitWindow();
    ui->progressBar->setHidden(true);
    ui->previewWidget->setCollapseTitle(true);
    ui->outputFormatComboBox->setCurrentIndex(1);   // set default JPEG 100%
    ui->codecComboBox->setCurrentIndex(1);          // set default codec to YUYV

    connect(ui->refreshButton,            SIGNAL(released()), this, SLOT(refresh()));
    connect(ui->capturePushButton,        SIGNAL(released()), this, SLOT(capture()));
    connect(ui->stopPushButton,           SIGNAL(released()), this, SLOT(stopCapture()));
    connect(ui->captureAdvancePushButton, SIGNAL(released()), this, SLOT(captureAndAdvance()));
    connect(ui->outDirButton,             SIGNAL(released()), this, SLOT(outputDir()));

    connect(ui->focusButton, SIGNAL(released()), &mFocusDialog, SLOT(show()));
    connect(ui->focusButton, SIGNAL(released()), &mFocusDialog, SLOT(raise()));

    connect(ui->rotaryControlPushButton, SIGNAL(released()), &mRotaryDialog, SLOT(show()));
    connect(ui->rotaryControlPushButton, SIGNAL(released()), &mRotaryDialog, SLOT(raise()));

    /* OK need to check if it gets deleted on table cleanup */
    connect(ui->cameraTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(tableClick(int, int)));

    ui->manipulatorCaptureButton->setVisible(false);
    mRuningManipulator = false;

    mCapSettingsDialog = new CapSettingsDialog();

    mCaptureMapper = new QSignalMapper();
    connect(mCaptureMapper, SIGNAL(mapped(int)), this, SLOT(newCaptureFrame(int)));
}

uint qHash(const ImageCaptureInterface::CameraFormat &format)
{
    return ((format.width * 1233434 + format.height) * 67234456567 + format.fps);
}

bool operator==(const ImageCaptureInterface::CameraFormat &format1, const ImageCaptureInterface::CameraFormat &format2)
{
    if  ((format1.width  == format2.width ) &&
         (format1.height == format2.height) &&
         (format1.fps    == format2.fps   ))
    {
        return true;
    }
    return false;
}

bool operator < (const ImageCaptureInterface::CameraFormat &format1, const ImageCaptureInterface::CameraFormat &format2)
{
    if (format1.height < format2.height) return true;
    if (format1.height > format2.height) return false;

    if (format1.width  < format2.width ) return true;
    if (format1.width  > format2.width ) return false;

    if (format1.fps    < format2.fps   ) return true;
    if (format1.fps    > format2.fps   ) return false;
    return false;
}

bool operator > (const ImageCaptureInterface::CameraFormat &format1, const ImageCaptureInterface::CameraFormat &format2)
{
    if (format1.height > format2.height) return true;
    if (format1.height < format2.height) return false;

    if (format1.width  > format2.width ) return true;
    if (format1.width  < format2.width ) return false;

    if (format1.fps    > format2.fps   ) return true;
    if (format1.fps    < format2.fps   ) return false;
    return false;
}

void PhotostationCaptureDialog::refresh()
{
    mCapSettingsDialog->setCaptureInterface(NULL);
    delete_safe(mPreviewInterface);

    /* Loading will be here for a while */
    QSettings settings(DEFAULT_FILENAME, QSettings::IniFormat);

    ui->heightSpinBox  ->setValue       (settings.value("height"    , ui->heightSpinBox->value()).toInt());
    ui->widthSpinBox   ->setValue       (settings.value("width"     , ui->widthSpinBox ->value()).toInt());
    ui->fpsSpinBox     ->setValue       (settings.value("fps"       , ui->fpsSpinBox   ->value()).toInt());
    ui->codecComboBox  ->setCurrentIndex(settings.value("codecIdx"  , ui->codecComboBox->currentIndex()).toInt());
  //ui->formatsComboBox->setCurrentIndex(settings.value("formatsIdx", ui->formatsComboBox->currentIndex()).toInt()); // it must be done later!

    /* Capturing */
    ui->skipFramesSpinBox     ->setValue(settings.value("skipFrames", ui->skipFramesSpinBox->value()).toInt());
    ui->stationNameLineEdit   ->setText (settings.value("nameSP"    , ui->stationNameLineEdit->text()).toString());
    ui->fileNamePrefixLineEdit->setText (settings.value("prefix"    , ui->fileNamePrefixLineEdit->text()).toString());
    ui->angleStepSpinBox      ->setValue(settings.value("angleStep" , ui->angleStepSpinBox->value()).toDouble());
    ui->outDirLineEdit        ->setText (settings.value("pathDir"   , ui->outDirLineEdit->text()).toString());

    mRotaryDialog.mScriptsPath = ui->outDirLineEdit->text();

    ui->previewWidget->loadFromQSettings(DEFAULT_FILENAME, "preview");

    mCameraInfos.clear();
    QMap<ImageCaptureInterface::CameraFormat, int> formatsList;
    vector<string> cameras;

    ImageCaptureInterface::getAllCameras(cameras);

    for (unsigned i = 0; i < cameras.size(); ++i)
    {
        mCameraInfos.push_back(CameraInfo());
        CameraInfo &info = mCameraInfos[i];
        info.listNumber = i;
        info.systemId = cameras[i];

        //qDebug() << "Found camera id:" <<  cameras[i].c_str();

        ImageCaptureInterface *camera = createCameraCapture(cameras[i], false);    // false - don't process any error
        if (camera == NULL)
            continue;

        int number = 0;
        ImageCaptureInterface::CameraFormat *formats = NULL;
        camera->getFormats(&number, formats);
        for (int j = 0; j < number; j++)
        {
            ImageCaptureInterface::CameraFormat &format = formats[j];
            if (formatsList.contains(format)) {
                formatsList[format]++;
            } else {
                formatsList.insert(format, 1);
            }
        }
        delete[] formats;

        info.serialId = camera->getDeviceSerial();
        delete_safe(camera);
    }

    ui->formatsComboBox->clear();
    QMapIterator<ImageCaptureInterface::CameraFormat, int> it(formatsList);

    disconnect(ui->formatsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(newFormatSelected(int)));
    while (it.hasNext())
    {
        it.next();
        QVariantList list;
        const ImageCaptureInterface::CameraFormat& fmt = it.key();
        list.append(fmt.width);
        list.append(fmt.height);
        list.append(fmt.fps);
        ui->formatsComboBox->addItem(QString("%1 x %2 : %3fps - %4").arg(fmt.width).arg(fmt.height).arg(fmt.fps).arg(it.value()), list);
    }
    ui->formatsComboBox->setCurrentIndex(settings.value("formatsIdx", 0).toInt());

    connect(ui->formatsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(newFormatSelected(int)));

    /* We need to fill height and width according to the camera support */

    ui->cameraTableWidget->clearContents();
    ui->cameraTableWidget->setRowCount(0);

    for (unsigned i = 0; i < cameras.size(); i++)
    {
        CameraInfo &info = mCameraInfos[i];
        /*Prepare additinal ids */
        std::string serial = info.serialId;
        std::string prefix = "SER_";

        if (HelperUtils::startsWith(serial, "-")) {
            serial = info.systemId;
            prefix = "POS_";
        }
        info.paramsStoreId = prefix + serial;



        ui->cameraTableWidget->setRowCount(ui->cameraTableWidget->rowCount() + 1);
        QTableWidgetItem *systemId = new QTableWidgetItem(QString("%1").arg(info.systemId.c_str()));

        systemId->setData(Qt::ToolTipRole, QString::fromStdString(info.serialId));

        ui->cameraTableWidget->setItem(i, COLUMN_SYS_ID, systemId);
        QComboBox* comboBox = new QComboBox();
        ui->cameraTableWidget->setCellWidget(i, COLUMN_CAM_ID, comboBox);
        for (unsigned j = 0; j < cameras.size(); j++)
        {
            comboBox->insertItem(comboBox->count(), QString("Camera %1").arg(j + 1));
        }
        comboBox->insertItem(comboBox->count(), "Unassigned");


        settings.beginGroup(QString::fromStdString(info.paramsStoreId));

        int index = settings.value("num", i).toInt();
        comboBox->setCurrentIndex(index);

        QCheckBox* checkBox = new QCheckBox();
        checkBox->setChecked(index < (int)cameras.size());
        ui->cameraTableWidget->setCellWidget(i, COLUMN_USE, checkBox);

        QTableWidgetItem* previewIcon = new QTableWidgetItem(QIcon(":/new/prefix1/play.png"), "");
        previewIcon->setFlags(previewIcon->flags() & (~Qt::ItemIsEditable));
        ui->cameraTableWidget->setItem(i, COLUMN_PREVIEW, previewIcon);

        QTableWidgetItem* settingsIcon = new QTableWidgetItem(QIcon(":/new/prefix1/cctv_camera.png"), "");
        settingsIcon->setFlags(settingsIcon->flags() & (~Qt::ItemIsEditable));
        ui->cameraTableWidget->setItem(i, COLUMN_SETTINGS, settingsIcon);

        bool restoreParameters = settings.value("restore", false).toBool();
        QCheckBox* checkBox1 = new QCheckBox();
        checkBox1->setChecked(restoreParameters);
        ui->cameraTableWidget->setCellWidget(i, COLUMN_SAVE_SETTINGS, checkBox1);

        settings.endGroup();

    }

    ui->cameraTableWidget->resizeRowsToContents();
    ui->cameraTableWidget->setColumnWidth(COLUMN_SYS_ID,       180);
    ui->cameraTableWidget->setColumnWidth(COLUMN_CAM_ID,       140);
    ui->cameraTableWidget->setColumnWidth(COLUMN_USE,           30);
    ui->cameraTableWidget->setColumnWidth(COLUMN_PREVIEW,       30);
    ui->cameraTableWidget->setColumnWidth(COLUMN_SETTINGS,      35);
    ui->cameraTableWidget->setColumnWidth(COLUMN_SAVE_SETTINGS, 30);

    mCamsScanned = true;
}


PhotostationCaptureDialog::~PhotostationCaptureDialog()
{
    /* Save parameters. Draft. */
    QSettings settings(DEFAULT_FILENAME, QSettings::IniFormat);

    settings.setValue("height"    , ui->heightSpinBox->value());
    settings.setValue("width"     , ui->widthSpinBox->value());
    settings.setValue("fps"       , ui->fpsSpinBox->value());
    settings.setValue("codecIdx"  , ui->codecComboBox->currentIndex());
    settings.setValue("formatsIdx", ui->formatsComboBox->currentIndex());

    /* Capturing */
    settings.setValue("skipFrames", ui->skipFramesSpinBox->value());
    settings.setValue("nameSP"    , ui->stationNameLineEdit->text());
    settings.setValue("prefix"    , ui->fileNamePrefixLineEdit->text());
    settings.setValue("angleStep" , ui->angleStepSpinBox->value());
    settings.setValue("pathDir"   , ui->outDirLineEdit->text());

    if (ui->cameraTableWidget->rowCount() != mCameraInfos.size())
    {
        SYNC_PRINT(("PhotostationCaptureDialog::~PhotostationCaptureDialog(): internal error"));
    }

    for (size_t i = 0; i < mCameraInfos.size(); i++ )
    {
        CameraInfo &info = mCameraInfos[i];

        QComboBox *comboBox  = static_cast<QComboBox *>(ui->cameraTableWidget->cellWidget(i, COLUMN_CAM_ID));
        QCheckBox *checkBox1 = static_cast<QCheckBox *>(ui->cameraTableWidget->cellWidget(i, COLUMN_SAVE_SETTINGS));

        settings.beginGroup(QString::fromStdString(info.paramsStoreId));
        settings.setValue("num"    , comboBox->currentIndex());
        settings.setValue("restore", checkBox1->isChecked());

        settings.endGroup();
    }

    ui->previewWidget->saveToQSettings(DEFAULT_FILENAME, "preview");
    delete ui;

    mCapSettingsDialog->setCaptureInterface(NULL);
    delete_safe(mPreviewInterface);

    delete_safe(mCapSettingsDialog);
    delete_safe(mCaptureMapper);

    for (int i = 0; i < mCaptureInterfaces.count(); i++)
    {
        CORE_ASSERT_TRUE_S(mCaptureInterfaces[i].camInterface == NULL);
        CORE_ASSERT_TRUE_S(mCaptureInterfaces[i].result == NULL);
    }

}

void PhotostationCaptureDialog::setNamer(AbstractImageNamer *namer)
{
    mNamer = namer;
}

void PhotostationCaptureDialog::setManipulator(AbstractManipulatorInterface *manipulator)
{
    if(mManipulatorCapturer != nullptr)
    {
        disconnect(ui->manipulatorCaptureButton, SIGNAL(released()), mManipulatorCapturer, SLOT(show()));
        disconnect(ui->manipulatorCaptureButton, SIGNAL(released()), mManipulatorCapturer, SLOT(raise()));

        disconnect(mManipulatorCapturer, SIGNAL(captureAndFinalize(bool)), this, SLOT(capture(bool)));
        disconnect(mManipulatorCapturer, SIGNAL(captureAtPosition(int)), this, SLOT(captureWithManipulator(int)));
        disconnect(mManipulatorCapturer, SIGNAL(manipulatorCaptureFinalise(bool)), this, SLOT(finalizeManipulatorCapture(bool)));
        disconnect(this, SIGNAL(captureFinished()), mManipulatorCapturer, SLOT(captureSimulatedStationFinshed()));
        disconnect(this, SIGNAL(waitingNextCapturePosition()), mManipulatorCapturer, SLOT(captureNextSimulatedCamera()));
    }
    mManipulatorCapturer = manipulator;
    ui->manipulatorCaptureButton->setVisible(mManipulatorCapturer != nullptr);
    if(mManipulatorCapturer != nullptr)
    {

        connect(ui->manipulatorCaptureButton, SIGNAL(released()), mManipulatorCapturer, SLOT(show()));
        connect(ui->manipulatorCaptureButton, SIGNAL(released()), mManipulatorCapturer, SLOT(raise()));

        connect(mManipulatorCapturer, SIGNAL(captureAndFinalize(bool)), this, SLOT(capture(bool)));
        connect(mManipulatorCapturer, SIGNAL(captureAtPosition(int)), this, SLOT(captureWithManipulator(int)));
        connect(mManipulatorCapturer, SIGNAL(manipulatorCaptureFinalise(bool)), this, SLOT(finalizeManipulatorCapture(bool)));
        connect(this, SIGNAL(captureFinished()), mManipulatorCapturer, SLOT(captureSimulatedStationFinshed()));
        connect(this, SIGNAL(waitingNextCapturePosition()), mManipulatorCapturer, SLOT(captureNextSimulatedCamera()));
    }
}

void PhotostationCaptureDialog::tableClick(int lineid, int colid)
{
    switch (colid) {
    case COLUMN_PREVIEW:
        if (!previewRunning()) {
            previewRequest(lineid);
        } else {
            previewStop();
        }
        break;
    case COLUMN_SETTINGS:
            cameraSettings(lineid);
        break;
    default:
        break;
    }
}

void PhotostationCaptureDialog::previewRequest(int lineid)
{
    //qDebug() << "PhotostationCaptureDialog::previewRequest(): Preview request";

    mCapSettingsDialog->setCaptureInterface(NULL);
    delete_safe(mPreviewInterface);

    string camSysId = ui->cameraTableWidget->item(lineid, COLUMN_SYS_ID)->text().toStdString();

    mPreviewInterface = createCameraCapture(camSysId);
    if (mPreviewInterface == NULL)
        return;
    mPreviewTableLine = lineid;

    connect(mPreviewInterface, SIGNAL(newFrameReady(frame_data_t)), this, SLOT(newPreviewFrame()));
    mPreviewInterface->startCapture();
    /* update the icon */

    QTableWidgetItem* previewIcon = new QTableWidgetItem(QIcon(":/new/prefix1/stop.png"), "");
    previewIcon->setFlags(previewIcon->flags() & (~Qt::ItemIsEditable));
    ui->cameraTableWidget->setItem(lineid, COLUMN_PREVIEW, previewIcon);

}

void PhotostationCaptureDialog::previewStop()
{
    SYNC_PRINT(("PhotostationCaptureDialog::previewStop()\n"));
    if (mPreviewTableLine != -1) {
         QCheckBox *checkBox1 = static_cast<QCheckBox *>(ui->cameraTableWidget->cellWidget(mPreviewTableLine, COLUMN_SAVE_SETTINGS));
         SYNC_PRINT(("PhotostationCaptureDialog::previewStop(): save settings flag: %s\n", checkBox1->isChecked() ? "on" : "off"));

         if (checkBox1->isChecked())
         {
             mCapSettingsDialog->saveToQSettings(DEFAULT_FILENAME, QString::fromStdString(mCameraInfos[mPreviewTableLine].paramsStoreId) + "_settings", false);
         }
    }

    mCapSettingsDialog->setCaptureInterface(NULL);
    delete_safe(mPreviewInterface);

    /* update the icon */
    if (mPreviewInterface >= 0 && mPreviewTableLine < ui->cameraTableWidget->rowCount())
    {
        QTableWidgetItem* previewIcon = new QTableWidgetItem(QIcon(":/new/prefix1/play.png"), "");
        previewIcon->setFlags(previewIcon->flags() & (~Qt::ItemIsEditable));
        ui->cameraTableWidget->setItem(mPreviewTableLine, COLUMN_PREVIEW, previewIcon);
    }
    mPreviewTableLine = -1;

}

bool PhotostationCaptureDialog::previewRunning()
{
    return (mPreviewInterface != NULL);
}

void PhotostationCaptureDialog::cameraSettings(int lineid)
{
    if (mPreviewInterface != NULL)
    {
        QCheckBox *checkBox1 = static_cast<QCheckBox *>(ui->cameraTableWidget->cellWidget(lineid, COLUMN_SAVE_SETTINGS));
        bool shouldRestore = checkBox1->isChecked();

        mCapSettingsDialog->clearDialog();
        mCapSettingsDialog->setCaptureInterface(mPreviewInterface);
        if (shouldRestore) {
            mCapSettingsDialog->loadFromQSettings(DEFAULT_FILENAME, QString::fromStdString(mCameraInfos[mPreviewTableLine].paramsStoreId) + "_settings", false);
        }
        mCapSettingsDialog->show();
    }
}

void PhotostationCaptureDialog::newPreviewFrame()
{
    // qDebug() << "PhotostationCaptureDialog::newPreviewFrame():Called";

    PreciseTimer time = PreciseTimer::currentTime();
    /* This protects the events form flooding input queue */
    static bool flushEvents = false;
    if (flushEvents) {        
        return;
    }
    flushEvents = true;
    /* Flush all events in queue */
    QCoreApplication::processEvents();
    flushEvents = false;

    // qDebug() << "PhotostationCaptureDialog::newPreviewFrame():Flood protection took:" << (time.usecsToNow() / 1000.0) << "ms";
    time = PreciseTimer::currentTime();

    /**
     * By the time we process notification, mPreviewInterface could be already destroyed
     **/
    if (mPreviewInterface == NULL)
        return;

    ImageCaptureInterface::FramePair pair = mPreviewInterface->getFrameRGB24();

    //qDebug() << "PhotostationCaptureDialog::newPreviewFrame(): grabbing pair took:" << (time.usecsToNow() / 1000.0) << "ms";
    time = PreciseTimer::currentTime();

    if (pair.rgbBufferLeft != NULL)
    {        
        QImage *preview = toQImage(pair.rgbBufferLeft);
        // qDebug("PhotostationCaptureDialog::newPreviewFrame():RGB image of [%d x %d]", preview->width(), preview->height()) ;
        ui->previewWidget->setImage(QSharedPointer<QImage>(preview));
    }
    else
    {
        //qDebug("PhotostationCaptureDialog::newPreviewFrame(): NULL frame received");
        L_DEBUG_P("NULL frame received");
    }

    // Estimate focus for the current image that has been set
    if (ui->focusComputeOnCheckBox->isChecked()){
        QRect rc = ui->previewWidget->getInputRect();
        int x1 = rc.left();
        int y1 = rc.top();
        int x2 = rc.right();
        int y2 = rc.bottom();

        FocusEstimator::Result res = FocusEstimator::calc(pair.rgbBufferLeft, x1, y1, x2, y2);
        ui->focusedLabel->setText(QString("Focus score: %1 out of %2").arg(res.score).arg(res.fullScore));
        mFocusDialog.addGraphPoint("Focus score", res.score, true);
        mFocusDialog.update();
    }

    //qDebug() << "PhotostationCaptureDialog::newPreviewFrame(): Updateing widget took:" << (time.usecsToNow() / 1000.0) << "ms";
    time = PreciseTimer::currentTime();

    pair.freeBuffers();
    //qDebug() << "PhotostationCaptureDialog::newPreviewFrame(): Free took:" << (time.usecsToNow() / 1000.0) << "ms";
}

void PhotostationCaptureDialog::newFormatSelected(int num)
{
    L_DEBUG_P("new formatId: %d", num);

    if (num >= ui->formatsComboBox->count())
        return;

    QVariantList format = ui->formatsComboBox->itemData(num).toList();
    if (format.length() != 3)
        return;

    ui->widthSpinBox->setValue (format[0].toInt());
    ui->heightSpinBox->setValue(format[1].toInt());
    ui->fpsSpinBox->setValue   (format[2].toInt());
}

void PhotostationCaptureDialog::newCaptureFrame(int camId)
{
    /* This protects the events from flooding input queue */
    static bool flushEvents = false;
    if (flushEvents)
        return;

    flushEvents = true;
    /* Flush all events in queue */
    QCoreApplication::processEvents();
    flushEvents = false;

    if (camId >= mCaptureInterfaces.count() || mCaptureInterfaces[camId].isFilled())
        return;

    /* Add frame skip */
    int startSkip = ui->skipFramesSpinBox->value();
    if (startSkip == 0)
    {
        ui->progressBar->setValue((camId + 1) * ui->progressBar->maximum() / mCaptureInterfaces.size());
    }
    else if (mCaptureInterfaces[camId].toSkip > 0)
    {
        mCaptureInterfaces[camId].toSkip--;
        int current = (camId + 1) * startSkip - mCaptureInterfaces[camId].toSkip;
        int total   = startSkip * mCaptureInterfaces.size();
        ui->progressBar->setValue(current * ui->progressBar->maximum() / total);
        return;
    }

    CameraDescriptor &descr = mCaptureInterfaces[camId];
    /* This could happen beacause of the old notifications */
    if (descr.camInterface == NULL)
    {
        L_INFO_P("Frame arrived after camera shutdown from %d", camId);
        return;
    }

    ImageCaptureInterface::FramePair pair = descr.camInterface->getFrameRGB24();
    if (pair.rgbBufferLeft == NULL) {
        L_ERROR_P("Unexpected zero rgbBuffer from camera %d", camId);
        pair.freeBuffers();
        return;
    }

    CORE_ASSERT_TRUE_S(!mCaptureInterfaces[camId].isFilled());

    mCaptureInterfaces[camId].result = toQImage(pair.rgbBufferLeft);
    pair.freeBuffers();
    clearSignalMappings(camId);
    disconnect(descr.camInterface, SIGNAL(newFrameReady(frame_data_t)), mCaptureMapper, SLOT(map()));
    delete_safe(descr.camInterface);

    /* This logic would propably change. We are starting so far  */

    for (int i = 0; i < mCaptureInterfaces.count(); i++)
    {
        if (!mCaptureInterfaces[i].isFilled())
        {
             QTimer::singleShot(ui->captureDelaySpinBox->value(), this, SLOT(initateNewFrame()));
             return;
        }
    }

    finalizeCapture(true);
}

void PhotostationCaptureDialog::initateNewFrame()
{
    for (int i = 0; i < mCaptureInterfaces.count(); i++)
    {
        if (!mCaptureInterfaces[i].isFilled())
        {
             mCaptureInterfaces[i].camInterface->startCapture();


             /* Put setting... it's not clear */
             QCheckBox *checkBox1 = static_cast<QCheckBox *>(ui->cameraTableWidget->cellWidget(mCaptureInterfaces[i].info.listNumber, COLUMN_SAVE_SETTINGS));
             bool shouldRestore = checkBox1->isChecked();

             mCapSettingsDialog->clearDialog();
             mCapSettingsDialog->setCaptureInterface(mCaptureInterfaces[i].camInterface);
             qDebug("PhotostationCaptureDialog::initateNewFrame(): %s should restore = %d", mCaptureInterfaces[i].info.serialId.c_str(), shouldRestore);
             if (shouldRestore) {
                 mCapSettingsDialog->loadFromQSettings(DEFAULT_FILENAME, QString::fromStdString(mCaptureInterfaces[i].info.paramsStoreId) + "_settings", false);
             }


             return;
        }
    }
}

void PhotostationCaptureDialog::capture(bool shouldAdvance, int positionShift)
{
    qDebug() << "PhotostationCaptureDialog::capture(" << shouldAdvance << ", " << positionShift << ")";

    mAdvanceAfterSave = shouldAdvance;

    mCapSettingsDialog->setCaptureInterface(NULL);
    delete_safe(mPreviewInterface);

    ui->capturePushButton       ->setEnabled(false);
    ui->captureAdvancePushButton->setEnabled(false);
    ui->stopPushButton          ->setEnabled(true);
    ui->refreshButton           ->setEnabled(false);
    ui->manipulatorCaptureButton->setEnabled(false);

    ui->progressBar->setHidden(false);
    ui->progressBar->setValue(0);

    mIsCalibrationMode = mRotaryDialog.isVisible() && mRotaryDialog.positions.size() != 0;
    ui->angleSpinBox->setEnabled(!mIsCalibrationMode);
    ui->angle2SpinBox->setEnabled(!mIsCalibrationMode);
    ui->angleStepSpinBox->setEnabled(!mIsCalibrationMode);
    L_INFO_P("table calibrationMode: %d  position: %d/%d", mIsCalibrationMode, mRotaryDialog.selected, mRotaryDialog.positions.size());

    if(mCaptureInterfaces.size() > 0)
        qDebug() << "Som trash in intrefaces collection detected!";

    /* Ok... create interfaces for all cameras */
    for (int lineid = 0; lineid < ui->cameraTableWidget->rowCount(); lineid++)
    {
        QComboBox *comboBox = static_cast<QComboBox *>(ui->cameraTableWidget->cellWidget(lineid, COLUMN_CAM_ID));
        QCheckBox *checkBox = static_cast<QCheckBox *>(ui->cameraTableWidget->cellWidget(lineid, COLUMN_USE));

        if (!checkBox->isChecked())                             // = not used
            continue;
        if (comboBox->currentIndex() == comboBox->count() - 1)  // = unassigned
            continue;

        string camSysId = ui->cameraTableWidget->item(lineid, COLUMN_SYS_ID)->text().toStdString();

        CameraDescriptor camDesc;
        camDesc.info  = mCameraInfos[lineid];
        camDesc.camId = comboBox->currentIndex();
        camDesc.toSkip = ui->skipFramesSpinBox->value();
        camDesc.result = NULL;
        camDesc.camInterface = createCameraCapture(camSysId);
        if (camDesc.camInterface == NULL)
            continue;

        mCaptureInterfaces.append(camDesc);
        int mappingIndex = mCaptureInterfaces.count() - 1;
        CORE_ASSERT_TRUE_S(mCaptureMapper->mapping(mappingIndex) == nullptr);
        connect(camDesc.camInterface, SIGNAL(newFrameReady(frame_data_t)), mCaptureMapper, SLOT(map()));
        mCaptureMapper->setMapping(camDesc.camInterface, mappingIndex);
    }

    if (positionShift > 0)
    {
        int shift = mCaptureInterfaces.size() * positionShift;
        for (auto& camDesc : mCaptureInterfaces)
            camDesc.camId += shift;
    }

    if (!mCaptureInterfaces.empty()) {
        initateNewFrame();
        // mCaptureInterfaces[0].camInterface->startCapture();
    }
    else {
        finalizeCapture(false);
    }
}

ImageCaptureInterfaceQt* PhotostationCaptureDialog::createCameraCapture(const string &devname, bool processError)
{
    const bool isRgb = true;
    ///bool compressed = ui->codecComboBox->currentIndex() == 0;

    int  h   = ui->heightSpinBox->value();
    int  w   = ui->widthSpinBox->value();
    int  fps = ui->fpsSpinBox->value();

    // TODO: use compressed YUYV, MJPG,... !

    ImageCaptureInterfaceQt *camera = ImageCaptureInterfaceQt::fabric(devname, h, w, fps, isRgb);

    ImageCaptureInterface::CapErrorCode result = camera->initCapture();
    ImageCaptureInterface::CameraFormat actualFormat;
    camera->getCurrentFormat(actualFormat);

    if (!processError)
        return camera;

    if (result != ImageCaptureInterface::SUCCESS_1CAM)
    {
        QMessageBox::information(this, "Camera Error", QString("Couldn't open the camera <%1>").arg(camera->getInterfaceName()));
        delete_safe(camera);
        return nullptr;
    }

    if ((!!actualFormat) && !(actualFormat == ImageCaptureInterface::CameraFormat(h, w, fps)))
    {
        int formatIdx;
        for (formatIdx = 0; formatIdx < ui->formatsComboBox->count(); ++formatIdx)
        {
            QVariantList format = ui->formatsComboBox->itemData(formatIdx).toList();
            if (format.length() != 3)
                break;

            if (actualFormat == ImageCaptureInterface::CameraFormat(format[1].toInt(), format[0].toInt(), format[2].toInt()))
                break;
        }

        w = actualFormat.width;
        h = actualFormat.height;
        fps = actualFormat.fps;

        QVariantList format = ui->formatsComboBox->itemData(formatIdx).toList();
        if(format.length() == 3)
        {
            ui->formatsComboBox->setCurrentIndex(formatIdx);
        }
        else
        {
            L_INFO_P("Camera <%s> format set to unexpected value.", QSTR_DATA_PTR(camera->getInterfaceName()));
        }
        L_INFO_P("camera <%s>: new format is: %dx%d @%d", QSTR_DATA_PTR(camera->getInterfaceName()), w, h, fps);
    }

    return camera;
}

void PhotostationCaptureDialog::clearSignalMappings(int i)
{

    QObject *mapper = mCaptureMapper->mapping(i);
    while(mapper != nullptr)
    {
        mCaptureMapper->removeMappings(mapper);
        mapper = mCaptureMapper->mapping(i);
    }
}

void PhotostationCaptureDialog::stopCapture()
{
    for (int i = 0; i < mCaptureInterfaces.count(); i++)
    {
        clearSignalMappings(i);
        delete_safe(mCaptureInterfaces[i].camInterface);
    }
    finalizeCapture(false);

    mCapSettingsDialog->setCaptureInterface(NULL);
    delete_safe(mPreviewInterface);
}

void PhotostationCaptureDialog::finalizeCapture(bool isOk)
{
    if (mIsCalibrationMode)
    {
        if (ui->stationNameLineEdit->text() == "A") {
            mRotaryDialog.selected = 0;
        }

        if (mRotaryDialog.selected < mRotaryDialog.positions.size())
        {
            const CameraLocationAngles& angles = mRotaryDialog.positions[mRotaryDialog.selected];

            ui->angleSpinBox->setValue(radToDeg(angles.roll()));
            ui->angle2SpinBox->setValue(radToDeg(angles.pitch()));
        }
    }

    QStringList failedSaves;
    /* Save images here */
    for (int i = 0; i < mCaptureInterfaces.count(); i++)
    {
        if (isOk && mNamer != NULL)
        {
            QString path   = ui->outDirLineEdit->text();
            QString prefix = ui->fileNamePrefixLineEdit->text();
            QString metaInfo;

            // prefix shouldn't have our keyword "SP"
            if (prefix.indexOf("SP") >= 0) {
                QMessageBox::warning(this, "Bad filename series prefix:", prefix);
                prefix.replace("SP", "sp");
            }

            if (mIsCalibrationMode)
            {
                char buf[256];
                snprintf2buf(buf, "%03d_%03ddeg"            // roll_pitch in degrees
                    , roundSign(ui->angleSpinBox->value())
                    , roundSign(ui->angle2SpinBox->value()));

                metaInfo = buf;
            }
            else {
                // TODO: add here planned features
                //metaInfo = QString::number(ui->angleSpinBox->value()) + "deg";
            }
            CORE_ASSERT_TRUE_S(mCaptureInterfaces[i].isFilled());

            std::string strPath = path.toStdString();

            std::string name = mNamer->nameForImage(
                ui->stationNameLineEdit->text().toStdString()
                , mCaptureInterfaces[i].camId
                , metaInfo.toStdString()
                , (AbstractImageNamer::FileType)ui->outputFormatComboBox->currentIndex()
                , &strPath
                , prefix.toStdString()
                );
            path = strPath.c_str();

            bool saveOk = mCaptureInterfaces[i].result->save(path, NULL
                            , (ui->outputFormatComboBox->currentIndex() == 0) ? 85 : 100);
            if (!saveOk) {
                failedSaves.append(path);
            }
            L_INFO_P("<%s> %ssaved", QSTR_DATA_PTR(path), saveOk ? "" : "not ");
        }

        delete_safe(mCaptureInterfaces[i].result);
    }

    for (int i = 0; i < mCaptureInterfaces.count(); i++) {
        CORE_ASSERT_TRUE_S(mCaptureInterfaces[i].camInterface == NULL);
        CORE_ASSERT_TRUE_S(mCaptureInterfaces[i].result == NULL);
    }
    mCaptureInterfaces.clear();

    if (mAdvanceAfterSave)
    {
        if (mIsCalibrationMode)
        {
            mRotaryDialog.executeAndIncrement();

            if (mRotaryDialog.selected < mRotaryDialog.positions.size())
            {
                const CameraLocationAngles& angles = mRotaryDialog.positions[mRotaryDialog.selected];

                ui->angleSpinBox->setValue(radToDeg(angles.roll()));
                ui->angle2SpinBox->setValue(radToDeg(angles.pitch()));
            }
        }
        else
        {
            ui->angleSpinBox->setValue(ui->angleSpinBox->value() + ui->angleStepSpinBox->value());
        }

        QString spName = ui->stationNameLineEdit->text();
        QString spName2 = spName;
        for (int i = 0; i < spName.length(); i++)
        {
            if (spName[i] > 'Z') spName[i] = 'Z';
            if (spName[i] < 'A') spName[i] = 'A';
        }
        if (spName != spName2) {
            QMessageBox::warning(this, spName2, "Bad PhotoStation point name!");
        }

        bool carry = true;
        for (int i = spName.length() - 1; i >= 0; i--)
        {
            if (spName[i] != 'Z') {
                spName[i] = QChar(spName[i].unicode() + 1);
                carry = false;
                break;
            }
            spName[i] = 'A';
        }
        if (carry) {
            spName = QString("A") + spName;
        }
        ui->stationNameLineEdit->setText(spName);
    }

    if (!failedSaves.isEmpty()) {
        QMessageBox::warning(this, "Error saving following files:", failedSaves.join(" "));
    }

    SYNC_PRINT(("PhotostationCaptureDialog::finalizeCapture(_): mRunnungManipulator=%s\n", mRuningManipulator ? "true" : "false"));

    if (!mRuningManipulator)
    {
        ui->capturePushButton       ->setEnabled(true);
        ui->captureAdvancePushButton->setEnabled(true);
        ui->manipulatorCaptureButton->setEnabled(true);

        ui->progressBar->setHidden(true);
        ui->progressBar->setValue(0);
        emit captureFinished();
    }
    else
    {
        emit waitingNextCapturePosition();
    }
}

void PhotostationCaptureDialog::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Space)
    {
        captureAndAdvance();
    }
}


void PhotostationCaptureDialog::showEvent(QShowEvent *event)
{
    QDialog::showEvent(event);
    if (!mCamsScanned) {
        refresh();
    }
}

void PhotostationCaptureDialog::captureAndAdvance()
{
    capture(true);
}

void PhotostationCaptureDialog::captureWithManipulator(int manipulatorPosition)
{
    SYNC_PRINT(("PhotostationCaptureDialog::captureWithManipulator(%d)\n", manipulatorPosition));

    mAdvanceAfterSave = false;
    mRuningManipulator = true;

    capture(false, manipulatorPosition);
}

void PhotostationCaptureDialog::finalizeManipulatorCapture(bool advance)
{
    SYNC_PRINT(("PhotostationCaptureDialog::finalizeManipulatorCapture: %s\n", advance ? "true" : "false"));
    mAdvanceAfterSave = advance;
    mRuningManipulator = false;

    finalizeCapture(false);
}

void PhotostationCaptureDialog::outputDir()
{
    QString path = ui->outDirLineEdit->text();

    QString pathNew = QFileDialog::getExistingDirectory(this, "Choose an output folder", path);

    if (pathNew.isEmpty() || pathNew == path)
        return;

    ui->outDirLineEdit->setText(pathNew);

    mRotaryDialog.mScriptsPath = pathNew;
}
