/**
 * \file TopViCapture.cpp
 * \brief Add Comment Here
 *
 * \date March 15, 2017
 * \author kat
 */

#define __STDC_FORMAT_MACROS
//#include <inttypes.h>
#undef __STDC_FORMAT_MACROS

#include <fcntl.h>
//#include <unistd.h>
#include <stdlib.h>
#include <QtCore/QRegExp>
#include <QtCore/QString>
#include <QtCore/QtCore>

#include "global.h"

#include "rgb24Buffer.h"
#include "g12Image.h"

#include "g12Image.h"
#include "ppmLoader.h"
#include "converters/debayer.h"

#include "topViCapture.h"
#include "topViDeviceDescriptor.h"

#ifdef PROFILE_DEQUEUE
#define TRACE_DEQUEUE(X) printf X
#else
#define TRACE_DEQUEUE(X)
#endif

#ifdef DEEP_TRACE_ON
#define DEEP_TRACE(X) SYNC_PRINT (X)
#else
#define DEEP_TRACE(X)
#endif

static const char *errorFileName = "tpv_20170414T164331_cam0_0.pgm";

TopViDeviceDescriptor TopViCaptureInterface::device;

int TopViCaptureInterface::topViTrace(int result, const char *prefix)
{
    if (result != 0)
    {
        char * str = 0;
        if (prefix == NULL)
            SYNC_PRINT(("TopViCaptureInterface: function failed with error code %d. Error string: %s\n", result, str));
        else
            SYNC_PRINT(("%s: error code %d. Error string: %s\n", prefix, result, str));

    }

    return result;
}

TopViCaptureInterface::TopViCaptureInterface(string _devname) :
    sync(NO_SYNC),
    ftpSpin(this)
{
    SYNC_PRINT(("TopViCaptureInterface: TopVi capture is using now\n"));
    setConfigurationString(_devname);
    device.init(0, this);
}

TopViCaptureInterface::TopViCaptureInterface(string _devname, int /*h*/, int /*w*/, int /*fps*/, bool isRgb) :
    sync(NO_SYNC),
    ftpSpin(this)
{
    SYNC_PRINT(("TopViCaptureInterface: TopVi capture is using now (constructor with parameters)\n"));
    setConfigurationString(_devname, isRgb);
    device.init(0, this);
}

int TopViCaptureInterface::activateFtpLoader(string links) {
    int result = false;
    protectActivate.lock();
    if (!shouldActivateFtpSpinThread) {
        //TODO: address is hardcode!!!
        ftpSpin.ftpLoader.init(ftpSpin.ftpLoader.addr, links);
        shouldActivateFtpSpinThread = true;
        SYNC_PRINT(("TopViCaptureInterface: replyCallback activate ftp loader success\n"));
        result = true;
    }
    else {
        SYNC_PRINT(("TopViCaptureInterface: replyCallback can't activate ftp loader\n"));
        result = false;
    }
    protectActivate.unlock();
    return result;
}

int TopViCaptureInterface::replyCallback(TopViGrillCommand *cmd){

    int result = false;

    string reply(cmd->reply->replyStr);

    SYNC_PRINT(("TopViCaptureInterface: replyCallback return answer for GUI: %s\n", cmd->reply->replyStr));

    switch (cmd->reply->cmdName) {
        case TPV_GRAB:
            result = activateFtpLoader(reply);
            break;
        default:
            SYNC_PRINT(("TopViCaptureInterface: do nothing, result is OK for %s\n", tpvCmdName(cmd->cmdName)));
            result = true;
    }

    return result;
}

int TopViCaptureInterface::setConfigurationString(string _devname, bool isRgb)
{
    CORE_UNUSED(isRgb);

    interfaceName = _devname;
    SYNC_PRINT(("TopViCaptureInterface: configuration string is %s\n", _devname.c_str()));
    this->devname = _devname;

    return 0;
}

TopViCaptureInterface::~TopViCaptureInterface()
{
    SYNC_PRINT(("TopViCaptureInterface(%s): request for killing the threads\n", QSTR_DATA_PTR(getInterfaceName())));

     while (ftpSpin.isRunning()) {
       SYNC_PRINT(("TopViCaptureInterface: waiting for the ftp thread cancelation...\n"));
       ftpSpinRunning.lock();
       shouldStopFtpSpinThread = true;
       ftpSpinRunning.unlock();
       QThread::usleep(200);
    }

    printf("TopViCaptureInterface: Closing TopVi capture...\n");
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::startCapture()
{
    SYNC_PRINT((" TopViCaptureInterface::startCapture(): called\n"));
    if (!ftpSpin.isRunning()) {
        ftpSpinRunning.lock();
        shouldStopFtpSpinThread = false;
        ftpSpinRunning.unlock();
        SYNC_PRINT(("TopVi is starting ftp loader\n"));
        ftpSpin.start();
    }

    SYNC_PRINT(("Ftp loader should be started\n"));

    device.grab(this);

    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::getFormats(int *num, ImageCaptureInterface::CameraFormat *&formats)
{
    SYNC_PRINT(("TopViCaptureInterface::getFormats()\n"));
    vector<ImageCaptureInterface::CameraFormat> cameraFormats;

    cameraFormats.push_back(ImageCaptureInterface::CameraFormat(2748, 3840, 6));
    //TODO: add format for preview

    *num = (int)cameraFormats.size();
    formats = new ImageCaptureInterface::CameraFormat[cameraFormats.size()];
    for (unsigned i = 0; i < cameraFormats.size(); i ++)
    {
        formats[i] = cameraFormats[i];
    }
    return ImageCaptureInterface::SUCCESS;
}

bool TopViCaptureInterface::getCurrentFormat(ImageCaptureInterface::CameraFormat &format)
{
    CORE_UNUSED(format);
    SYNC_PRINT(("TopViCaptureInterface::getCurrentFormat() is not supported\n"));
    return ImageCaptureInterface::SUCCESS;
}


void TopViCaptureInterface::getAllCameras(vector<string> &cameras)
{
    device.getCamerasSysId(cameras);
}

QString TopViCaptureInterface::getInterfaceName()
{
    return QString("tpv://") + QString(interfaceName.c_str());
}

string TopViCaptureInterface::getDeviceSerial(int /*num*/)
{
    //SYNC_PRINT(("TopViCaptureInterface::getDeviceSerial():called\n"));
    //TODO: doesn't better solution !!!
    string camSysId = interfaceName.substr(interfaceName.find_first_of('_') + 1, 1);
    int camId = QString(camSysId.c_str()).toInt();
    std::string toReturn = to_string(camId);

    SYNC_PRINT(("TopviCaptureInterface::getDeviceSerial():returning <%s>\n", toReturn.c_str()));
    return toReturn;
}

void TopViCaptureInterface::FtpSpinThread::run()
{
    SYNC_PRINT(("TopViCaptureInterface::ftpSpinThread(): ftp loader thread running\n"));

    while (capInterface->ftpSpinRunning.tryLock()) {
        if (capInterface->shouldActivateFtpSpinThread) {
            if (ftpLoader.inited) {
                SYNC_PRINT(("FtpSpinThread::run(): ftp get starting\n"));
                //ftpLoader.makeTest();
                ftpLoader.getFile();

                /* Now exchange the buffer that is visible from */
                capInterface->protectFrame.lock();
                frame_data_t frameData;
                frameData.timestamp = capInterface->currentFrame->usecsTimeStamp();
                capInterface->notifyAboutNewFrame(frameData);
                capInterface->protectFrame.unlock();
            }
            else {
                SYNC_PRINT(("FtpSpinThread::run(): ftp not inited now, will try later\n"));
            }

            capInterface->protectActivate.lock();
            capInterface->shouldActivateFtpSpinThread = false;
            capInterface->protectActivate.unlock();
            SYNC_PRINT(("FtpSpinThread::run(): ftp get finished\n"));
        }
        capInterface->ftpSpinRunning.unlock();

        if (capInterface->shouldStopFtpSpinThread)
        {
            SYNC_PRINT(("Break command to ftp spin thread received"));
            break;
        }

        usleep(200);
    }
    SYNC_PRINT(("TopViCaptureInterface: ftpLoader thread finished\n"));
}


ImageCaptureInterface::CapErrorCode TopViCaptureInterface::initCapture()
{
    SYNC_PRINT(("TopViCaptureInterface::initCapture(): called\n"));

    int res = ImageCaptureInterface::SUCCESS_1CAM;

    CapErrorCode result = (CapErrorCode)((bool) res);

    SYNC_PRINT(("TopViCaptureInterface::initCapture(): returning %d\n", result));
    return result;
}

TopViCaptureInterface::FramePair  TopViCaptureInterface::getFrame(){
    FramePair pair(NULL, NULL);
    SYNC_PRINT(("TopViCaptureInterface::getFrame(): called\n"));

    QString name = getActiveFileName();

    SYNC_PRINT(("TopViCaptureInterface::getFrame(): show file %s\n", name.toLatin1().constData()));

    if (name.isNull() || name.isEmpty()) {
        name = QString(errorFileName);
    }

    MetaData meta;
    G12Buffer *bayer = PPMLoader().g12BufferCreateFromPGM(name.toStdString(), &meta);
    if (bayer == NULL) {
        SYNC_PRINT(("Can't' open bayer file: %s", name.toLatin1().constData()));
    }
    else {
        DebayerMethod::DebayerMethod methodId = DebayerMethod::NEAREST;
        int bayerPos = 1;
        int shift = -4;
        uint16_t mask = 0xFFFF;

        Debayer d(bayer, meta["bits"][0], &meta, bayerPos);
        RGB48Buffer* input = new RGB48Buffer(bayer->h, bayer->w, false);
        d.toRGB48(methodId, input);

        RGB24Buffer *result = new RGB24Buffer(input->getSize());
        for (int i = 0; i < result->h; i ++)
        {
            for (int j = 0; j < result->w; j ++)
            {
                RGB48Buffer::InternalElementType colorIn = input->element(i, j);
                RGBColor colorOut;
                if (shift >= 0) {
                    colorOut.r() = (colorIn.r() & mask) << shift;
                    colorOut.g() = (colorIn.g() & mask) << shift;
                    colorOut.b() = (colorIn.b() & mask) << shift;
                }
                if (shift < 0) {
                    colorOut.r() = (colorIn.r() & mask) >> (-shift);
                    colorOut.g() = (colorIn.g() & mask) >> (-shift);
                    colorOut.b() = (colorIn.b() & mask) >> (-shift);
                }
                result->element(i,j) = colorOut;
            }
        }
        pair.rgbBufferLeft = result;
        delete_safe(input);
    }
    SYNC_PRINT(("TopViCaptureInterface::getFrame(): finished\n"));
    return pair;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::queryCameraParameters(CameraParameters &params)
{
    CORE_UNUSED(params);
    SYNC_PRINT(("TopViCaptureInterface: queryCameraParameters is not supported for TopVi interface yet.\n"));
    return ImageCaptureInterface::SUCCESS;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::setCaptureProperty(int id, int value)
{
    CORE_UNUSED(id);
    CORE_UNUSED(value);
    SYNC_PRINT(("TopViCaptureInterface:: setCaptureProperty is not supported for TopVi interface yet.\n"));
    //TODO: may be good place for get output dir path
    return ImageCaptureInterface::FAILURE;
}

ImageCaptureInterface::CapErrorCode TopViCaptureInterface::getCaptureProperty(int id, int *value)
{
    CORE_UNUSED(id);
    CORE_UNUSED(value);
    SYNC_PRINT(("TopViCaptureInterface:: getCaptureProperty is not supported for TopVi interface yet.\n"));
    return ImageCaptureInterface::FAILURE;
}
