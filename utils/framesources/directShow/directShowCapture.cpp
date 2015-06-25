/**
 * \file directShowCapture.cpp
 * \brief Capture camera video stream using DirectShow
 *
 * \date Jul 15, 2010
 * \author apimenov
 */

#include <QtCore/QRegExp>
#include <QtCore/QString>

#include <sstream>

#include "global.h"

#include "preciseTimer.h"
#include "directShowCapture.h"
#include "mjpegDecoderLazy.h"
#include "cameraControlParameters.h"

const char* DirectShowCaptureInterface::codec_names[] =
{
        "yuv",
        "rgb",
        "mjpeg",
        "mjpeg fast decoder"
};

DirectShowCaptureInterface::DirectShowCaptureInterface(string _devname, int h, int w, int fps, bool isRgb)
{
    this->devname = QString("%1:1/%2:yuyv:%3x%4").arg(_devname.c_str()).arg(fps).arg(w).arg(h).toStdString();
    deviceID[Frames::LEFT_FRAME] = atoi(_devname.c_str());
    deviceID[Frames::RIGHT_FRAME] = -1;

    compressed = DirectShowCameraDescriptor::UNCOMPRESSED_YUV;

    for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; i++)
    {
        format[i].type   = CAP_YUV;
        format[i].height = h;
        format[i].width  = w;
        format[i].fps    = fps;
    }
}

DirectShowCaptureInterface::DirectShowCaptureInterface(string _devname, ImageCaptureInterface::CameraFormat inFormat, bool isRgb)
{
    this->devname = QString("%1:1/%2:yuyv:%3x%4").arg(_devname.c_str()).arg(inFormat.fps).arg(inFormat.width).arg(inFormat.height).toStdString();
    deviceID[Frames::LEFT_FRAME] = atoi(_devname.c_str());
    deviceID[Frames::RIGHT_FRAME] = -1;

    compressed = DirectShowCameraDescriptor::UNCOMPRESSED_YUV;

    for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; i++)
    {
        format[i].type   = CAP_YUV;
        format[i].height = inFormat.height;
        format[i].width  = inFormat.width;
        format[i].fps    = inFormat.fps;
    }

}

DirectShowCaptureInterface::DirectShowCaptureInterface(string _devname)
{
    this->devname = _devname;

    //     Group Number                   1       2 3        4 5      6       7 8       9       10     11        1213     14
    QRegExp deviceStringPattern(QString("^([^,:]*)(,([^:]*))?(:(\\d*)/(\\d*))?((:mjpeg)|(:yuyv)|(:rgb)|(:fjpeg))?(:(\\d*)x(\\d*))?$"));
    static const int Device1Group     = 1;
    static const int Device2Group     = 3;
    static const int FpsNumGroup      = 5;
    static const int FpsDenumGroup    = 6;
    static const int CompressionGroup = 7;
    static const int WidthGroup       = 13;
    static const int HeightGroup      = 14;

    printf ("Input string %s\n", _devname.c_str());
    QString qdevname(_devname.c_str());
    int result = deviceStringPattern.indexIn(qdevname);
    if (result == -1)
    {
        printf("Error in device string format:%s\n", _devname.c_str());
        return;
    }
    printf (
    "Parsed data:\n"
    "  | - Device 1=%s\n"
    "  | - Device 2=%s\n"
    "  | - FPS %s/%s\n"
    "  | - Size [%sx%s]\n"
    "  \\ - Compressing: %s\n",
    deviceStringPattern.cap(Device1Group).toLatin1().constData(),
    deviceStringPattern.cap(Device2Group).toLatin1().constData(),
    deviceStringPattern.cap(FpsNumGroup).toLatin1().constData(),
    deviceStringPattern.cap(FpsDenumGroup).toLatin1().constData(),
    deviceStringPattern.cap(WidthGroup).toLatin1().constData(),
    deviceStringPattern.cap(HeightGroup).toLatin1().constData(),
    deviceStringPattern.cap(CompressionGroup).toLatin1().constData());

    deviceID[Frames::LEFT_FRAME ] = deviceStringPattern.cap(Device1Group).isEmpty() ? -1 : deviceStringPattern.cap(Device1Group).toInt();
    deviceID[Frames::RIGHT_FRAME] = deviceStringPattern.cap(Device2Group).isEmpty() ? -1 : deviceStringPattern.cap(Device2Group).toInt();

    bool err;
    int fpsnum = deviceStringPattern.cap(FpsNumGroup).toInt(&err);
    if (!err) fpsnum = 1;

    int fpsdenum = deviceStringPattern.cap(FpsDenumGroup).toInt(&err);
    if (!err) fpsdenum = 10;

    int width = deviceStringPattern.cap(WidthGroup).toInt(&err);
    if (!err || width <= 0) width = 800;

    int height = deviceStringPattern.cap(HeightGroup).toInt(&err);
    if (!err || height <= 0) height = 600;

    compressed = DirectShowCameraDescriptor::UNCOMPRESSED_YUV;
    int formatId = CAP_YUV;
    if (!deviceStringPattern.cap(CompressionGroup).isEmpty())
    {
       if        (!deviceStringPattern.cap(CompressionGroup).compare(QString(":rgb"))) {
           compressed = DirectShowCameraDescriptor::UNCOMPRESSED_RGB;
           formatId = CAP_RGB;
       } else if (!deviceStringPattern.cap(CompressionGroup).compare(QString(":mjpeg"))) {
           compressed = DirectShowCameraDescriptor::COMPRESSED_JPEG;
           formatId = CAP_MJPEG;
       } else if (!deviceStringPattern.cap(CompressionGroup).compare(QString(":fjpeg"))) {
           compressed = DirectShowCameraDescriptor::COMPRESSED_FAST_JPEG;
           formatId = CAP_MJPEG;
       }
    }

    for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; i++)
    {
        printf("Capture %s device: DShow %d\n", Frames::getEnumName((Frames::FrameSourceId)i), deviceID[i]);
    }
    printf("Format is: %s\n", codec_names[compressed]);

    /* TODO: Make cycle here */

    for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; i++)
    {
        format[i].type   = formatId;
        format[i].height = height;
        format[i].width  = width;
        format[i].fps    = (int)((double)fpsdenum / fpsnum);
    }


    skippedCount = 0;
    isRunning = false;
}

ImageCaptureInterface::CapErrorCode DirectShowCaptureInterface::initCapture()
{
    for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; i++)
    {
        cameras[Frames::LEFT_FRAME ].deviceHandle =  deviceID[i] >= 0 ? DirectShowCapDll_initCapture(deviceID[i])  : -1;
    }

    for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; i++)
    {
        if (!isCorrectDeviceHandle(i))
        {
            continue;
        }
        DirectShowCapDll_setFormat(cameras[i].deviceHandle, &format[i]);
        DirectShowCapDll_setFrameCallback(cameras[i].deviceHandle, this, DirectShowCaptureInterface::callback);
    }

    printf("Real Formats:\n");
    for (int i = 0; i < Frames::MAX_INPUTS_NUMBER; i++)
    {
        DirectShowCapDll_printSimpleFormat(&format[i]); printf("\n");
    }

    return ImageCaptureInterface::SUCCESS;
}

void DirectShowCaptureInterface::callback (void *thiz, DSCapDeviceId dev, FrameData data)
{
    //printf("Received new frame in a callback\n");
    DirectShowCaptureInterface *pInterface = static_cast<DirectShowCaptureInterface *>(thiz);
    pInterface->memberCallback(dev, data);
}

ALIGN_STACK_SSE void DirectShowCaptureInterface::memberCallback(DSCapDeviceId dev, FrameData data)
{
    //SYNC_PRINT(("Received new frame in a member %d\n", dev));
    protectFrame.lock();

    DirectShowCameraDescriptor *camera = NULL;
    if      (cameras[0].deviceHandle == dev)
        camera = &cameras[0];
    else if (cameras[1].deviceHandle == dev)
        camera = &cameras[1];
    else
        goto exit;

    {
        PreciseTimer timer = PreciseTimer::currentTime();
        camera->gotBuffer = true;
        camera->timestamp = (data.timestamp + 5) / 10;
        delete camera->buffer;

        if (data.format.type == CAP_YUV)
        {
            camera->buffer = new G12Buffer(data.format.height, data.format.width, false);
            camera->buffer->fillWithYUYV((uint16_t *)data.data);
        }
        else if (data.format.type == CAP_MJPEG)
        {
            MjpegDecoderLazy lazyDecoder;
            camera->buffer = lazyDecoder.decode((unsigned char *)data.data);
        }
        else if (data.format.type == CAP_RGB)
        {
            // Take this somewhere else
            camera->buffer = new G12Buffer(data.format.height, data.format.width, false);
            int w = camera->buffer->w;
            int h = camera->buffer->h;
            for (int i = 0; i < h; i++)
            {
                uint8_t  *rgbData = ((uint8_t *)data.data) + 3 * (h - i - 1) * w;
                uint16_t *greyData = &(camera->buffer->element(i,0));
                for (int j = 0; j < w; j++)
                {
                    uint16_t r = rgbData[0];
                    uint16_t g = rgbData[1];
                    uint16_t b = rgbData[2];
                    *greyData = (11 * r + 16 * g + 5 * b) >> 1;
                    rgbData +=3;
                    greyData+=1;
                }
            }
        } else {
            camera->buffer = new G12Buffer(data.format.height, data.format.width, false);
        }

        camera->decodeTime = timer.usecsToNow();
        /* If both frames are in place */

        if (cameras[0].gotBuffer && cameras[1].gotBuffer)
        {
            cameras[0].gotBuffer = false;
            cameras[1].gotBuffer = false;


            CaptureStatistics stats;
            int64_t desync =  cameras[0].timestamp - cameras[1].timestamp;
            stats.values[CaptureStatistics::DESYNC_TIME] = desync > 0 ? desync : -desync;
            stats.values[CaptureStatistics::DECODING_TIME] = cameras[0].decodeTime + cameras[1].decodeTime;
            if (lastFrameTime.usecsTo(PreciseTimer()) != 0)
            {
                stats.values[CaptureStatistics::INTERFRAME_DELAY] = lastFrameTime.usecsToNow();
            }
            lastFrameTime = PreciseTimer::currentTime();

            frame_data_t frameData;
            frameData.timestamp = cameras[0].timestamp / 2 + cameras[1].timestamp / 2;
            newFrameReady(frameData);
            newStatisticsReady(stats);
        } else {
            emit newImageReady();
            skippedCount++;
        }
    }
exit:
    protectFrame.unlock();
}


ImageCaptureInterface::FramePair DirectShowCaptureInterface::getFrame()
{
    protectFrame.lock();
        FramePair result;
        if (cameras[0].buffer != NULL)
        {
            result.bufferLeft = new G12Buffer(cameras[0].buffer);
        }
        if (cameras[1].buffer != NULL)
        {
            result.bufferRight = new G12Buffer(cameras[1].buffer);
        }
        result.timeStampLeft  = cameras[0].timestamp;
        result.timeStampRight = cameras[1].timestamp;
    protectFrame.unlock();
    return result;
}

ImageCaptureInterface::CapErrorCode DirectShowCaptureInterface::startCapture()
{
    isRunning = true;
    if (isCorrectDeviceHandle(0))
    {
        DirectShowCapDll_start(cameras[0].deviceHandle);
    }
    if (isCorrectDeviceHandle(1))
    {
        DirectShowCapDll_start(cameras[1].deviceHandle);
    }
    //spin.start();
    return ImageCaptureInterface::SUCCESS;
}

DirectShowCaptureInterface::~DirectShowCaptureInterface()
{
    if (!isRunning)
        return;
        /*
            Callback is set safe and synchroniously,
            so after the call of the callback reset, we will never again
            recive a frame. So it will be safe to destroy the object
        */
    if (isCorrectDeviceHandle(0))
    {
        DirectShowCapDll_setFrameCallback(cameras[0].deviceHandle, NULL, NULL);
        DirectShowCapDll_stop(cameras[0].deviceHandle);
    }
    if (isCorrectDeviceHandle(1))
    {
        DirectShowCapDll_setFrameCallback(cameras[1].deviceHandle, NULL, NULL);
        DirectShowCapDll_stop(cameras[1].deviceHandle);
    }
    isRunning = false;
}



ImageCaptureInterface::CapErrorCode DirectShowCaptureInterface::queryCameraParameters(CameraParameters &parameters)
{
    if (isCorrectDeviceHandle(0))
    {
        cameras[0].queryCameraParameters(parameters);
    }
    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode DirectShowCaptureInterface::setCaptureProperty(int id, int value)
{
    if (isCorrectDeviceHandle(0))
    {
        cameras[0].setCaptureProperty(id, value);
    }
    if (isCorrectDeviceHandle(1))
    {
        cameras[1].setCaptureProperty(id, value);
    }
    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode DirectShowCaptureInterface::getCaptureProperty(int id, int *value)
{
    if (isCorrectDeviceHandle(0))
    {
        cameras[0].getCaptureProperty(id, value);
    }
    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode DirectShowCaptureInterface::getCaptureName(QString &value)
{
    if (!isCorrectDeviceHandle(0))
        return ImageCaptureInterface::FAILURE;

    char *name = NULL;
    if (DirectShowCapDll_deviceName(cameras[0].deviceHandle, &name))
        return ImageCaptureInterface::FAILURE;

    value = name;
    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode DirectShowCaptureInterface::getFormats(int *num, CameraFormat *&format)
{
    if (!isCorrectDeviceHandle(0))
        return ImageCaptureInterface::FAILURE;
    DirectShowCapDll_getFormatNumber(cameras[0].deviceHandle, num);

    int number = *num;
    CaptureTypeFormat* captureTypeFormats = new CaptureTypeFormat[number];

    delete format;
    format = new CameraFormat[number];
    DirectShowCapDll_getFormats(cameras[0].deviceHandle, number, captureTypeFormats);
    for (int i = 0; i < number; i ++)
    {
        format[i].fps    = captureTypeFormats[i].fps;
        format[i].width  = captureTypeFormats[i].width;
        format[i].height = captureTypeFormats[i].height;
    }

    delete[] captureTypeFormats;

    return ImageCaptureInterface::SUCCESS;
}

DirectShowCaptureInterface::CapErrorCode DirectShowCaptureInterface::getDeviceName(int num, QString &name)
{
    if (num < 0 || num >= 2)
    {
        return ImageCaptureInterface::FAILURE;
    }
    //     Group Number                   1       2 3        4 5      6       7 8       9       10     11        1213     14
    QRegExp deviceStringPattern(QString("^([^,:]*)(,([^:]*))?(:(\\d*)/(\\d*))?((:mjpeg)|(:yuyv)|(:rgb)|(:fjpeg))?(:(\\d*)x(\\d*))?$"));
    static const int Device1Group     = 1;
    static const int Device2Group     = 3;
    QString qdevname(devname.c_str());
    int result = deviceStringPattern.indexIn(qdevname);
    if (result == -1)
    {
        printf("Error in device string format:%s\n", devname.c_str());
        return ImageCaptureInterface::FAILURE;
    }
    if (num == 0)
    {
        name = deviceStringPattern.cap(Device1Group);
    }
    else
    {
        name = deviceStringPattern.cap(Device2Group);
    }
    return ImageCaptureInterface::SUCCESS;
}

void DirectShowCaptureInterface::getAllCameras(vector<string> &cameras)
{
    int num = 0;
    DirectShowCapDll_devicesNumber(&num);
    for (int i = 0; i < num; i++)
    {
        std::ostringstream s;
        s << i;
        cameras.push_back(s.str());
    }

}

bool DirectShowCaptureInterface::isCorrectDeviceHandle(int cameraNum)
{
    return cameras[cameraNum].deviceHandle >= 0;
}
