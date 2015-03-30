#include "syncCamerasCaptureInterface.h"


SyncCamerasCaptureInterface::SyncCamerasCaptureInterface(string _devname) :
    currentPair(NULL)
  , formatH(480)
  , formatW(640)
  , spin(this)
{
    setConfigurationString(_devname);
    camera = new USBSyncCam();
}

SyncCamerasCaptureInterface::~SyncCamerasCaptureInterface()
{
     cout << "SyncCamerasCaptureInterface::Request for killing the thread" << endl;

     SYNC_PRINT(("SyncCamerasCaptureInterface::Stopping camera\n"));

     shouldStopSpinThread = true;
     bool result = spinRunning.tryLock(1000);

     if (result) {
         SYNC_PRINT(("SyncCamerasCaptureInterface::Camera thread killed\n"));
     } else {
         SYNC_PRINT(("SyncCamerasCaptureInterface::Unable to exit Camera thread\n"));
     }

     //TODO: Move to StopCapturing???
     camera->StopCapturing();
     camera->DeinitCapturing();
     delete camera;
     protectFrame.lock();
     delete currentPair.bufferLeft;
     delete currentPair.bufferRight;
     protectFrame.unlock();

}

int SyncCamerasCaptureInterface::setConfigurationString(string _devname)
{
    this->devname = _devname;

    return 0;
}

//TODO: check types

static G12Buffer* decodeBayer(uint16_t * data, int h, int w )
{
    G12Buffer *toReturn = new G12Buffer(h, w);
    for (int i = 0; i < h; i += 2)
    {
        for (int j = 0; j < w; j += 2)
        {
            uint32_t r  = (uint32_t)data[ (i    ) * w + (j + 1)] >> 4;
            uint32_t g1 = (uint32_t)data[ (i    ) * w + (j    )] >> 4;
            uint32_t g2 = (uint32_t)data[ (i + 1) * w + (j + 1)] >> 4;
            uint32_t b  = (uint32_t)data[ (i + 1) * w + (j    )] >> 4;

            uint32_t mean = (r + g1 + g2 + b) / 4;

            toReturn->element(i    , j    ) = mean;
            toReturn->element(i    , j + 1) = mean;
            toReturn->element(i + 1, j    ) = mean;
            toReturn->element(i + 1, j + 1) = mean;

        }
    }
    return toReturn;
}

static RGB24Buffer* decodeBayerRGB(uint16_t * data, int h, int w )
{
    RGB24Buffer *toReturn = new RGB24Buffer(h, w);
    for (int i = 0; i < h; i += 2)
    {
        for (int j = 0; j < w; j += 2)
        {
            uint32_t g1  = ((uint32_t)data[ (i    ) * w + (j + 1)]) >> 8;
            uint32_t r   = ((uint32_t)data[ (i    ) * w + (j    )]) >> 8;
            uint32_t b   = ((uint32_t)data[ (i + 1) * w + (j + 1)]) >> 8;
            uint32_t g2  = ((uint32_t)data[ (i + 1) * w + (j    )]) >> 8;

            if (r > 255 || g1 > 255 || g2 > 255 || b > 255) {
                printf("trouble\n");
            }

            RGBColor color = RGBColor((uint8_t)r, (uint8_t)((g1 + g2) / 2), (uint8_t)b);
            toReturn->element(i    , j    ) = color;
            toReturn->element(i    , j + 1) = color;
            toReturn->element(i + 1, j    ) = color;
            toReturn->element(i + 1, j + 1) = color;

        }
    }
    return toReturn;
}



ImageCaptureInterface::FramePair SyncCamerasCaptureInterface::getFrame()
{
    FramePair result;

//    printf("getFrame:lock\n");

//#ifndef BAYER
//    result.bufferLeft = new G12Buffer(formatH, formatW, false);
//    result.bufferLeft->fillWithYUYV((uint16_t*)data->GetFrame(syncCam::EP_1)->GetBuffer());
//    result.bufferRight = new G12Buffer(formatH, formatW, false);
//    result.bufferRight->fillWithYUYV((uint16_t*)data->GetFrame(syncCam::EP_2)->GetBuffer());
//#else
    // TODO: Free from this copying


    protectFrame.lock();
    result.bufferLeft  = currentPair.rgbBufferLeft->toG12Buffer();
    result.bufferRight = currentPair.rgbBufferRight->toG12Buffer();
//#endif
    result.leftTimeStamp  = currentPair. leftTimeStamp;
    result.rightTimeStamp = currentPair.rightTimeStamp;

//    printf("getFrame:unlock, ts1: %d, ts2: %d\n", result.leftTimeStamp, result.rightTimeStamp);
    protectFrame.unlock();

    result.rgbBufferLeft = NULL;
    result.rgbBufferRight = NULL;

    return result;
}

ImageCaptureInterface::FramePair SyncCamerasCaptureInterface::getFrameRGB24()
{
    FramePair result;

//    printf("getFrame:lock\n");

//#ifndef BAYER
//    result.bufferLeft = new G12Buffer(formatH, formatW, false);
//    result.bufferLeft->fillWithYUYV((uint16_t*)data->GetFrame(syncCam::EP_1)->GetBuffer());
//    result.bufferRight = new G12Buffer(formatH, formatW, false);
//    result.bufferRight->fillWithYUYV((uint16_t*)data->GetFrame(syncCam::EP_2)->GetBuffer());
//#else
    // TODO: Free from this copying


    protectFrame.lock();
    result.bufferLeft  = currentPair.rgbBufferLeft ->toG12Buffer();
    result.bufferRight = currentPair.rgbBufferRight->toG12Buffer();

    result.rgbBufferLeft  = new RGB24Buffer(currentPair.rgbBufferLeft);
    result.rgbBufferRight = new RGB24Buffer(currentPair.rgbBufferRight);


//#endif
    result.leftTimeStamp  = currentPair.leftTimeStamp;
    result.rightTimeStamp = currentPair.rightTimeStamp;

//    printf("getFrame:unlock, ts1: %d, ts2: %d\n", result.leftTimeStamp, result.rightTimeStamp);
    protectFrame.unlock();
    return result;
}


ImageCaptureInterface::CapErrorCode SyncCamerasCaptureInterface::initCapture()
{
    shouldStopSpinThread = false;
    int res = camera->InitCapturing();
    if (res != 0)
        return ImageCaptureInterface::FAILURE;

    formatH = camera->GetHeight();
    formatW = camera->GetWidth();

    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode SyncCamerasCaptureInterface::startCapture()
{
    camera->StartCapturing();
    shouldStopSpinThread = false;
    spin.start(QThread::TimeCriticalPriority);

    return ImageCaptureInterface::SUCCESS;
}

SensParam SyncCamerasCaptureInterface::ConvertParamId(int id)
{
    SensParam cameraParam = PARAM_COUNT;

    //TODO: Enhance parameter mapping!
    switch (id)
    {
    case CameraParameters::GAIN:
        cameraParam = GLOBAL_ANALOG_GAIN;
        break;
    case CameraParameters::GAIN_MULTIPLIER:
        cameraParam = GLOBAL_ANALOG_MULT;
        break;
    case CameraParameters::GAIN_DIGITAL:
        cameraParam = GLOBAL_DIGITAL_GAIN;
        break;

    case CameraParameters::GAIN_BLUE:
        cameraParam = BLUE_ANALOG_GAIN;
        break;
    case CameraParameters::GAIN_BLUE_MULTIPLIER:
        cameraParam = BLUE_ANALOG_MULT;
        break;
    case CameraParameters::GAIN_BLUE_DIGITAL:
        cameraParam = BLUE_DIGITAL_GAIN;
        break;

    case CameraParameters::GAIN_GREEN:
        cameraParam = GREEN1_ANALOG_GAIN;
        break;
    case CameraParameters::GAIN_GREEN_MULTIPLIER:
        cameraParam = GREEN1_ANALOG_MULT;
        break;
    case CameraParameters::GAIN_GREEN_DIGITAL:
        cameraParam = GREEN1_DIGITAL_GAIN;
        break;

    case CameraParameters::GAIN_RED:
        cameraParam = RED_ANALOG_GAIN;
        break;
    case CameraParameters::GAIN_RED_MULTIPLIER:
        cameraParam = RED_ANALOG_MULT;
        break;
    case CameraParameters::GAIN_RED_DIGITAL:
        cameraParam = RED_DIGITAL_GAIN;
        break;

    case CameraParameters::GAIN_GREEN2:
        cameraParam = GREEN2_ANALOG_GAIN;
        break;
    case CameraParameters::GAIN_GREEN2_MULTIPLIER:
        cameraParam = GREEN2_ANALOG_MULT;
        break;
    case CameraParameters::GAIN_GREEN2_DIGITAL:
        cameraParam = GREEN2_DIGITAL_GAIN;
        break;
    case CameraParameters::SHUTTER_WIDTH_U:
        cameraParam = SHUTTER_WIDTH_UP;
        break;
    case CameraParameters::SHUTTER_WIDTH_L:
        cameraParam = SHUTTER_WIDTH_LOW;
        break;
    case CameraParameters::SHUTTER_DEL:
        cameraParam = SHUTTER_DELAY;
        break;
    default:
        //TODO: Add comment
        break;
    }

    return cameraParam;
}


ImageCaptureInterface::CapErrorCode SyncCamerasCaptureInterface::setCaptureProperty(int id, int value)
{
    SensParam cameraParam = ConvertParamId(id);

    printf("id: %d, sensParam: %d, value: %d\n", id, cameraParam, value);

    if (cameraParam == PARAM_COUNT)
    {
        printf("URA1\n");
        return ImageCaptureInterface::FAILURE;
    }

    if (!camera->SetParam(cameraParam, (uint16_t)value))
    {
        printf("URA2\n");
        return ImageCaptureInterface::FAILURE;
    }
    printf("URA3\n");

    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode SyncCamerasCaptureInterface::getCaptureProperty(int id, int *value)
{
    SensParam cameraParam = ConvertParamId(id);
    if (cameraParam == PARAM_COUNT)
        return ImageCaptureInterface::FAILURE;

    uint16_t res = 0;
    if (!camera->GetParam(cameraParam, res))
        return ImageCaptureInterface::FAILURE;

    *value = res;
    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode SyncCamerasCaptureInterface::queryCameraParameters(CameraParameters &parameter)
{

    //TODO: Set other options
    /* Gain */

    /* Global analog gain */
    CaptureParameter *param = &(parameter.mCameraControls[CameraParameters::GAIN]);
    param->setActive(true);
    param->setDefaultValue(0x08);
    param->setMinimum     (0x08);
    param->setMaximum     (0x3F);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* Global analog multiplier */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_MULTIPLIER]);
    param->setActive(true);
    param->setDefaultValue(0x00);
    param->setMinimum     (0x00);
    param->setMaximum     (0x01);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* Global digital gain*/
    param = &(parameter.mCameraControls[CameraParameters::GAIN_DIGITAL]);
    param->setActive(true);
    param->setDefaultValue(0x00);
    param->setMinimum     (0x00);
    param->setMaximum     (0x78);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);




    /* Green gain */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_GREEN]);
    param->setActive(true);
    param->setDefaultValue(0x08);
    param->setMinimum     (0x08);
    param->setMaximum     (0x3F);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* Green analog multiplier */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_GREEN_MULTIPLIER]);
    param->setActive(true);
    param->setDefaultValue(0x00);
    param->setMinimum     (0x00);
    param->setMaximum     (0x01);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* Green digital gain*/
    param = &(parameter.mCameraControls[CameraParameters::GAIN_GREEN_DIGITAL]);
    param->setActive(true);
    param->setDefaultValue(0x00);
    param->setMinimum     (0x00);
    param->setMaximum     (0x78);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);




    /* Blue analog gain */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_BLUE]);
    param->setActive(true);
    param->setDefaultValue(0x08);
    param->setMinimum     (0x08);
    param->setMaximum     (0x3F);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* Blue analog multiplier */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_BLUE_MULTIPLIER]);
    param->setActive(true);
    param->setDefaultValue(0x00);
    param->setMinimum     (0x00);
    param->setMaximum     (0x01);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* Blue digital gain */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_BLUE_DIGITAL]);
    param->setActive(true);
    param->setDefaultValue(0x00);
    param->setMinimum     (0x00);
    param->setMaximum     (0x78);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);




    /* Red analog gain */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_RED]);
    param->setActive(true);
    param->setDefaultValue(0x08);
    param->setMinimum     (0x08);
    param->setMaximum     (0x3F);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* Red analog gain */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_RED_MULTIPLIER]);
    param->setActive(true);
    param->setDefaultValue(0x00);
    param->setMinimum     (0x00);
    param->setMaximum     (0x01);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* Red digital gain */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_RED_DIGITAL]);
    param->setActive(true);
    param->setDefaultValue(0x00);
    param->setMinimum     (0x00);
    param->setMaximum     (0x78);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);


    /* Green2 analog gain */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_GREEN2]);
    param->setActive(true);
    param->setDefaultValue(0x08);
    param->setMinimum     (0x08);
    param->setMaximum     (0x3F);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* Green2 analog miltiplier */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_GREEN2_MULTIPLIER]);
    param->setActive(true);
    param->setDefaultValue(0x00);
    param->setMinimum     (0x00);
    param->setMaximum     (0x01);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* Green2 digital gain */
    param = &(parameter.mCameraControls[CameraParameters::GAIN_GREEN2_DIGITAL]);
    param->setActive(true);
    param->setDefaultValue(0x00);
    param->setMinimum     (0x00);
    param->setMaximum     (0x78);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* SHUTTER WIDTH UPPER */
    param = &(parameter.mCameraControls[CameraParameters::SHUTTER_WIDTH_U]);
    param->setActive(true);
    param->setDefaultValue(0x0000);
    param->setMinimum     (0x0000);
    param->setMaximum     (0xFFFF);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* SHUTTER WIDTH LOWER */
    param = &(parameter.mCameraControls[CameraParameters::SHUTTER_WIDTH_L]);
    param->setActive(true);
    param->setDefaultValue(0x0797);
    param->setMinimum     (0x0000);
    param->setMaximum     (0xFFFF);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);

    /* SHUTTER Delay */
    param = &(parameter.mCameraControls[CameraParameters::SHUTTER_DEL]);
    param->setActive(true);
    param->setDefaultValue(0x1FFF);
    param->setMinimum     (0x0000);
    param->setMaximum     (0xFFFF);
    param->setStep        (1);
    param->setIsMenu(true);
    //TODO: What daes it mean?
    //param->setAutoSupported(true);


    return ImageCaptureInterface::SUCCESS;
}


SyncCamerasCaptureInterface::SpinThread::SpinThread(SyncCamerasCaptureInterface *_interface) :
    interface(_interface)
{
}

void SyncCamerasCaptureInterface::SpinThread::run()
{
    printf("Spin thread is started\n");
    while (interface->spinRunning.tryLock())
    {

        syncCam::FramePair *data = NULL;

        data = interface->camera->GetFramePair();

            if (!data)
                printf("FAILURE: NULL\n");
    //        else
    //            printf("run: URA, ts1: %d, ts2: %d\n", data->GetFrame(EP_1)->getDataIndex(), data->GetFrame(EP_2)->getDataIndex());

            //TODO: make rotation of cameras

            unsigned char* left = data->GetFrame(syncCam::EP_1)->GetBuffer();
            unsigned char* right = data->GetFrame(syncCam::EP_2)->GetBuffer();

            if (left == NULL)
            {
                printf("FAILURE: left is NULL\n");
            }
            if (right == NULL)
            {
                printf("FAILURE: right is NULL\n");
            }

            RGB24Buffer *leftRGB24  = decodeBayerRGB((uint16_t*)left,  interface->formatH, interface->formatW);
            RGB24Buffer *rightRGB24 = decodeBayerRGB((uint16_t*)right, interface->formatH, interface->formatW);

            uint64_t leftTimeStamp = (uint64_t)data->GetFrame(syncCam::EP_1)->getDataIndex();
            uint64_t rightTimeStamp = (uint64_t)data->GetFrame(syncCam::EP_2)->getDataIndex();

        interface->camera->FreeFramePair();


        interface->protectFrame.lock();

            delete_safe(interface->currentPair.rgbBufferLeft);
            interface->currentPair.rgbBufferLeft  = leftRGB24;

            delete_safe(interface->currentPair.rgbBufferRight);
            interface->currentPair.rgbBufferRight = rightRGB24;

            delete_safe(interface->currentPair.bufferLeft);
            interface->currentPair.bufferLeft  = NULL;

            delete_safe(interface->currentPair.bufferRight);
            interface->currentPair.bufferRight = NULL;

            interface->currentPair.leftTimeStamp = leftTimeStamp;
            interface->currentPair.rightTimeStamp = rightTimeStamp;

        interface->protectFrame.unlock();

        frame_data_t frameData;
        frameData.timestamp = leftTimeStamp;

        interface->notifyAboutNewFrame(frameData);



        interface->spinRunning.unlock();
    }
}


