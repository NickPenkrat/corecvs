#include <QMessageBox>
#include <QFileDialog>

#include "g12Image.h"
#include "photostationCaptureDialog.h"
//#include "../projectFileNameing.h"

#include "ui_photostationCaptureDialog.h"

/* Temporary solution. This need to be hidden inside image capture interface */
#ifdef Q_OS_WIN
#ifdef WITH_DIRECTSHOW
#include "directShowCapture.h"
#define CAPTURE_INTERFACE DirectShowCaptureInterface
#endif
#else
#include "V4L2Capture.h"
#define CAPTURE_INTERFACE V4L2CaptureInterface
#endif

#include "qtHelper.h"
#include "log.h"
#include "focusEstimator.h"

#ifdef WITH_DIRECTSHOW
#include "directShow.h"
#endif

const QString PhotostationCaptureDialog::DEFAULT_FILENAME = "capture.ini";

PhotostationCaptureDialog::PhotostationCaptureDialog(QWidget *parent) :
    QDialog(parent),
    mCamsScanned(false),
    mPreviewInterface(NULL),
    mCapSettingsDialog(NULL),
    mCaptureMapper(NULL),
    mAdvanceAfterSave(false),
    ui(new Ui::PhotostationCaptureDialog),
    mNamer(NULL)
{
    setWindowFlags(Qt::Window);
    ui->setupUi(this);
    ui->previewWidget->setFitWindow();
    ui->progressBar->setHidden(true);
    ui->previewWidget->setCollapseTitle(true);
    ui->outputFormatComboBox->setCurrentIndex(1);   // set default JPEG 100%

    connect(ui->refreshButton,            SIGNAL(released()), this, SLOT(refresh()));
    connect(ui->capturePushButton,        SIGNAL(released()), this, SLOT(capture()));
    connect(ui->stopPushButton,           SIGNAL(released()), this, SLOT(stopCapture()));
    connect(ui->captureAdvancePushButton, SIGNAL(released()), this, SLOT(captureAndAdvance()));
    connect(ui->outDirButton,             SIGNAL(released()), this, SLOT(outputDir()));

    connect(ui->focusButton, SIGNAL(released()), &focusDialog, SLOT(show()));
    connect(ui->focusButton, SIGNAL(released()), &focusDialog, SLOT(raise()));

    connect(ui->rotaryControlPushButton, SIGNAL(released()), &rotaryDialog, SLOT(show()));
    connect(ui->rotaryControlPushButton, SIGNAL(released()), &rotaryDialog, SLOT(raise()));


    /* OK need to check if it gets deleted on table cleanup */
    connect(ui->cameraTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(tableClick(int, int)));


    mCapSettingsDialog = new CapSettingsDialog();
}

uint qHash(const ImageCaptureInterface::CameraFormat &format)
{
    return ((format.width * 1233434 +  format.height) * 67234456567 + format.fps);
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


    ui->heightSpinBox->setValue(settings.value("height", ui->heightSpinBox->value()).toInt());
    ui->widthSpinBox ->setValue(settings.value("width" , ui->widthSpinBox->value()) .toInt()) ;
    ui->fpsSpinBox   ->setValue(settings.value("fps"   , ui->fpsSpinBox->value())   .toInt());

    /* Capturing */
    ui->skipFramesSpinBox     ->setValue(settings.value("skipFrames", ui->skipFramesSpinBox->value()).toInt());
    ui->stationNameLineEdit   ->setText (settings.value("nameSP"    , ui->stationNameLineEdit->text()).toString());
    ui->fileNamePrefixLineEdit->setText (settings.value("prefix"    , ui->fileNamePrefixLineEdit->text()).toString());
    ui->angleStepSpinBox      ->setValue(settings.value("angleStep" , ui->angleStepSpinBox->value()).toDouble());
    ui->outDirLineEdit        ->setText (settings.value("pathDir"   , ui->outDirLineEdit->text()).toString());

    ui->previewWidget->loadFromQSettings(DEFAULT_FILENAME, "preview");

    QMap<ImageCaptureInterface::CameraFormat, int> formatsList;
    vector<string> cameras;
    vector<string> serials;

#ifdef Q_OS_WIN
#ifdef WITH_DIRECTSHOW
    CAPTURE_INTERFACE::getAllCameras(cameras);
#endif
#else
    CAPTURE_INTERFACE::getAllCameras(cameras);
#endif

    for (unsigned i = 0; i < cameras.size(); i ++)
    {
        qDebug() << "Camera found:" <<  cameras[i].c_str();

#ifdef Q_OS_WIN
#ifdef WITH_DIRECTSHOW
        ImageCaptureInterface *camera = new CAPTURE_INTERFACE(cameras[i], 480, 640, 10, true);
#endif
#else
        ImageCaptureInterface *camera = new CAPTURE_INTERFACE(cameras[i], 480, 640, 10, true);
#endif

        camera->initCapture();
        int number = 0;
        ImageCaptureInterface::CameraFormat *formats = NULL;
        camera->getFormats(&number, formats);
        for (int j = 0; j < number; j++)
        {
            ImageCaptureInterface::CameraFormat &format = formats[j];
            if (formatsList.contains(format)) {
                formatsList[format]++;
            } else {
                formatsList.insert(format,1);
            }
        }
        delete[] formats;


        serials.push_back(camera->getDeviceSerial());
        delete_safe (camera);
    }

    ui->formatsComboBox->clear();
    QMapIterator<ImageCaptureInterface::CameraFormat, int> it(formatsList);

    disconnect(ui->formatsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(newFormatSelected(int)));
    while (it.hasNext())
    {
        it.next();
        QVariantList list;
        list.append(it.key().width);
        list.append(it.key().height);
        list.append(it.key().fps);
        ui->formatsComboBox->addItem(QString("%1 x %2 : %3fps - %4").arg(it.key().width).arg(it.key().height).arg(it.key().fps).arg(it.value()), list);
    }
    connect(ui->formatsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(newFormatSelected(int)));

    /* We need to fill height and width according to the camera support */

    ui->cameraTableWidget->clearContents();
    ui->cameraTableWidget->setRowCount(0);

    for (unsigned i = 0; i < cameras.size(); i++)
    {
        ui->cameraTableWidget->setRowCount(ui->cameraTableWidget->rowCount() + 1);
        QTableWidgetItem *systemId = new QTableWidgetItem(QString("%1").arg(cameras[i].c_str()));

        if (i <= serials.size())
        {
            systemId->setData(Qt::ToolTipRole, QString::fromStdString(serials[i]));
        }
        ui->cameraTableWidget->setItem(i, COLUMN_SYS_ID, systemId);
        QComboBox* comboBox = new QComboBox();
        ui->cameraTableWidget->setCellWidget(i, COLUMN_PS_ID, comboBox);
        for (unsigned j = 0; j <  cameras.size(); j++)
        {
            comboBox->insertItem(comboBox->count(), QString("Camera %1").arg(j + 1));
        }


        QString serial = QString::fromStdString(serials[i]);
        QString prefix = "SER_";

        if (serial.startsWith("-")) {
            serial = QString::fromStdString(cameras[i]);
            prefix = "POS_";
        }

        int index = settings.value(prefix + serial, i).toInt();
        comboBox->insertItem(comboBox->count(), "Unassigned");
        comboBox->setCurrentIndex(index);

        QCheckBox* checkBox = new QCheckBox();
        checkBox->setChecked(true);
        ui->cameraTableWidget->setCellWidget(i, COLUMN_USE, checkBox);

        QTableWidgetItem* previewIcon = new QTableWidgetItem(QIcon(":/new/prefix1/play.png"), "");
        previewIcon->setFlags(previewIcon->flags() & (~Qt::ItemIsEditable));
        ui->cameraTableWidget->setItem(i, COLUMN_PREVIEW, previewIcon);

        QTableWidgetItem* settingsIcon = new QTableWidgetItem(QIcon(":/new/prefix1/cctv_camera.png"), "");
        settingsIcon->setFlags(settingsIcon->flags() & (~Qt::ItemIsEditable));
        ui->cameraTableWidget->setItem(i, COLUMN_SETTINGS, settingsIcon);
    }

    ui->cameraTableWidget->resizeRowsToContents();
    ui->cameraTableWidget->setColumnWidth(COLUMN_SYS_ID,  100);
    ui->cameraTableWidget->setColumnWidth(COLUMN_PS_ID,   140);
    ui->cameraTableWidget->setColumnWidth(COLUMN_USE,      30);
    ui->cameraTableWidget->setColumnWidth(COLUMN_PREVIEW,  30);
    ui->cameraTableWidget->setColumnWidth(COLUMN_SETTINGS, 30);

    mCamsScanned = true;
}


PhotostationCaptureDialog::~PhotostationCaptureDialog()
{
    /* Save parameters. Draft. */
    QSettings settings(DEFAULT_FILENAME, QSettings::IniFormat);

    settings.setValue("height", ui->heightSpinBox->value());
    settings.setValue("width" , ui->widthSpinBox->value());
    settings.setValue("fps"   , ui->fpsSpinBox->value());

    /* Capturing */
    settings.setValue("skipFrames", ui->skipFramesSpinBox->value());
    settings.setValue("nameSP"    , ui->stationNameLineEdit->text());
    settings.setValue("prefix"    , ui->fileNamePrefixLineEdit->text());    
    settings.setValue("angleStep" , ui->angleStepSpinBox->value());
    settings.setValue("pathDir"   , ui->outDirLineEdit->text());

    for (int i = 0; i < ui->cameraTableWidget->rowCount(); i++ )
    {
        QString serial = ui->cameraTableWidget->item(i, 0)->data(Qt::ToolTipRole).toString();
        QString prefix = "SER_";

        if (serial.startsWith("-")) {
            serial = ui->cameraTableWidget->item(i, 0)->data(Qt::DisplayRole).toString();
            prefix = "POS_";
        }

        QComboBox *comboBox = static_cast<QComboBox *>(ui->cameraTableWidget->cellWidget(i, COLUMN_PS_ID));
        settings.setValue(prefix + serial, comboBox->currentIndex());
    }

    ui->previewWidget->saveToQSettings(DEFAULT_FILENAME, "preview");
    delete ui;

    delete_safe(mCapSettingsDialog);
}

void PhotostationCaptureDialog::setNamer(AbstractImageNamer *namer)
{
    mNamer = namer;
}

void PhotostationCaptureDialog::tableClick(int lineid, int colid)
{
    switch (colid) {
    case COLUMN_PREVIEW:
            previewRequest(lineid);
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
    qDebug() << "PhotostationCaptureDialog::previewRequest(): Preview request";

    mCapSettingsDialog->setCaptureInterface(NULL);
    delete_safe(mPreviewInterface);

    mPreviewInterface = new CAPTURE_INTERFACE(
                ui->cameraTableWidget->item(lineid, COLUMN_SYS_ID)->text().toStdString(),
                ui->heightSpinBox->value(),
                ui->widthSpinBox->value(),
                ui->fpsSpinBox->value(),
                true);
    ImageCaptureInterface::CapErrorCode result = mPreviewInterface->initCapture();
    if (result != ImageCaptureInterface::SUCCESS_1CAM)
    {
        QMessageBox::information(this, "Can't open", "I can't open the camera");
        return;
    }
    connect(mPreviewInterface, SIGNAL(newFrameReady(frame_data_t)), this, SLOT(newPreviewFrame()));
    mPreviewInterface->startCapture();
}

void PhotostationCaptureDialog::cameraSettings(int /*lineid*/)
{
    if (mPreviewInterface != NULL)
    {
        mCapSettingsDialog->clearDialog();
        mCapSettingsDialog->setCaptureInterface(mPreviewInterface);
        mCapSettingsDialog->show();
    }
}

void PhotostationCaptureDialog::newPreviewFrame()
{
    //qDebug() << "PhotostationCaptureDialog::newPreviewFrame():Trace";

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

    //qDebug() << "PhotostationCaptureDialog::newPreviewFrame():Flood protection took:" << (time.usecsToNow() / 1000.0) << "ms";
    time = PreciseTimer::currentTime();

    /*By the time we process notification mPreviewInterface could be already destroyed */
    if (mPreviewInterface == NULL) {
        return;
    }

    ImageCaptureInterface::FramePair pair = mPreviewInterface->getFrameRGB24();

    //qDebug() << "PhotostationCaptureDialog::newPreviewFrame(): grabbing pair took:" << (time.usecsToNow() / 1000.0) << "ms";
    time = PreciseTimer::currentTime();

    int focusValue = corecvs::FocusEstimator::getScore(pair.rgbBufferLeft);
    ui->focusedLabel->setText(QString("Focus Score: %1").arg(focusValue));
    focusDialog.addGraphPoint("Focal Score", focusValue, true);
    focusDialog.update();

    if (pair.rgbBufferLeft != NULL)
    {
        ui->previewWidget->setImage(QSharedPointer<QImage>(toQImage(pair.rgbBufferLeft)));
    } else {
        qDebug() << "PhotostationCaptureDialog::newPreviewFrame():NULL frame received";
    }

    //qDebug() << "PhotostationCaptureDialog::newPreviewFrame(): Updateing widget took:" << (time.usecsToNow() / 1000.0) << "ms";
    time = PreciseTimer::currentTime();

    pair.freeBuffers();
    //qDebug() << "PhotostationCaptureDialog::newPreviewFrame(): Free took:" << (time.usecsToNow() / 1000.0) << "ms";
}

void PhotostationCaptureDialog::newFormatSelected(int num)
{
    qDebug() << "PhotostationCaptureDialog::newFormatSelected(" << num << ")";

    if (num >= ui->formatsComboBox->count())
    {
        return;
    }
    QVariantList format = ui->formatsComboBox->itemData(num).toList();
    if (format.length() != 3)
    {
        return;
    }
    ui->widthSpinBox->setValue (format[0].toInt());
    ui->heightSpinBox->setValue(format[1].toInt());
    ui->fpsSpinBox->setValue   (format[2].toInt());
}

void PhotostationCaptureDialog::newCaptureFrame(int camId)
{
    /* This protects the events from flooding input queue */
    static bool flushEvents = false;
    if (flushEvents) {
        return;
    }
    flushEvents = true;
    /* Flush all events in queue */
    QCoreApplication::processEvents();
    flushEvents = false;

    if (camId >= mCaptureInterfaces.count() || mCaptureInterfaces[camId].result != NULL)
    {
        return;
    }

    /* Add frame skip */
    if (mCaptureInterfaces[camId].toSkip > 0)
    {
        mCaptureInterfaces[camId].toSkip--;
        int startSkip = ui->skipFramesSpinBox->value();
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
        L_ERROR_P("Unexpected zero buffer form camera %d", camId);
        pair.freeBuffers();
        return;
    }

    mCaptureInterfaces[camId].result = toQImage(pair.rgbBufferLeft);
    pair.freeBuffers();
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
             return;
        }
    }
}


void PhotostationCaptureDialog::capture(bool shouldAdvance)
{
    mAdvanceAfterSave = shouldAdvance;

    mCapSettingsDialog->setCaptureInterface(NULL);
    delete_safe(mPreviewInterface);

    ui->capturePushButton       ->setEnabled(false);
    ui->captureAdvancePushButton->setEnabled(false);
    ui->stopPushButton          ->setEnabled(true);
    ui->refreshButton           ->setEnabled(false);

    ui->progressBar->setHidden(false);
    ui->progressBar->setValue(0);

    delete_safe(mCaptureMapper);
    mCaptureMapper = new QSignalMapper();
    connect(mCaptureMapper, SIGNAL(mapped(int)), this, SLOT(newCaptureFrame(int)));

    /* Ok... init all cameras */
    for (int lineid = 0; lineid < ui->cameraTableWidget->rowCount(); lineid++)
    {
        QComboBox *comboBox = static_cast<QComboBox *>(ui->cameraTableWidget->cellWidget(lineid, COLUMN_PS_ID));
        QCheckBox *checkBox = static_cast<QCheckBox *>(ui->cameraTableWidget->cellWidget(lineid, COLUMN_USE));

        if (!checkBox->isChecked()) {
            continue;
        }

        if (comboBox->currentIndex() == comboBox->count() - 1) {
            continue;
        }

        string str = ui->cameraTableWidget->item(lineid, COLUMN_SYS_ID)->text().toStdString();

        CameraDescriptor camDesc;
        camDesc.camId = comboBox->currentIndex();
        camDesc.camInterface = new CAPTURE_INTERFACE(
                        str,
                        ui->heightSpinBox->value(),
                        ui->widthSpinBox->value(),
                        ui->fpsSpinBox->value(),
                        true);
        camDesc.result = NULL;
        camDesc.toSkip = ui->skipFramesSpinBox->value();


        ImageCaptureInterface::CapErrorCode result = camDesc.camInterface->initCapture();
        if (result != ImageCaptureInterface::SUCCESS_1CAM)
        {
            QMessageBox::information(this, "Can't open", QString("I can`t open the camera %1").arg(str.c_str()));
            return;
        }
        mCaptureInterfaces.append(camDesc);
        connect(camDesc.camInterface, SIGNAL(newFrameReady(frame_data_t)), mCaptureMapper, SLOT(map()));
        mCaptureMapper->setMapping(camDesc.camInterface, mCaptureInterfaces.count() - 1);
    }

    if (!mCaptureInterfaces.empty())
    {
        mCaptureInterfaces[0].camInterface->startCapture();
    }
    else {
        finalizeCapture(false);
    }
}

void PhotostationCaptureDialog::stopCapture()
{
    for (int i = 0; i < mCaptureInterfaces.count(); i++)
    {
        delete_safe(mCaptureInterfaces[i].camInterface);
    }
    finalizeCapture(false);

    mCapSettingsDialog->setCaptureInterface(NULL);
    delete_safe(mPreviewInterface);
}

void PhotostationCaptureDialog::finalizeCapture(bool isOk)
{
    ui->capturePushButton       ->setEnabled(true);
    ui->captureAdvancePushButton->setEnabled(true);

    ui->progressBar->setHidden(true);
    ui->progressBar->setValue(0);

    QStringList failedSaves;
    /* Save images here */
    for (int i = 0; i < mCaptureInterfaces.count(); i++)
    {
        if (isOk && mNamer != NULL)
        {
            QString path   = ui->outDirLineEdit->text();
            QString prefix = ui->fileNamePrefixLineEdit->text();

            if (prefix.indexOf("SP") >= 0) {
                QMessageBox::warning(this, "Bad filename series prefix:", prefix);
                prefix.replace("SP", "sp");
            }

            QString name = mNamer->nameForImage(
                ui->stationNameLineEdit->text()
                , mCaptureInterfaces[i].camId
                , QString::number(ui->angleSpinBox->value()) + "deg"
                , (AbstractImageNamer::FileType)ui->outputFormatComboBox->currentIndex()
                , &path
                , prefix
                );

            bool saveOk = mCaptureInterfaces[i].result->save(path, NULL
                            , (ui->outputFormatComboBox->currentIndex() == 0) ? 85 : 100);
            if (!saveOk) {
                failedSaves.append(path);
            }
            L_INFO_P("File <%s> %ssaved", QSTR_DATA_PTR(path), saveOk ? "" : "not ");
        }

        delete_safe(mCaptureInterfaces[i].result);
    }

    mCaptureInterfaces.clear();

    if (mAdvanceAfterSave)
    {
        ui->angleSpinBox->setValue(ui->angleSpinBox->value() + ui->angleStepSpinBox->value());

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

void PhotostationCaptureDialog::outputDir()
{
    QString path = ui->outDirLineEdit->text();

    QString pathNew = QFileDialog::getExistingDirectory(this, "Choose an output folder", path);

    if (pathNew.isEmpty() || pathNew == path)
        return;

    ui->outDirLineEdit->setText(pathNew);
}
