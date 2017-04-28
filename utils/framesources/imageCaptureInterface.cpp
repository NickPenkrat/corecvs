/**
 * \file ImageCaptureInterface.cpp
 * \brief Add Comment Here
 *
 * \date Mar 14, 2010
 * \author alexander
 */
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <QtCore/QMetaType>

#include "global.h"

#include "imageCaptureInterface.h"
#include "cameraControlParameters.h"

#include "fileCapture.h"
#include "precCapture.h"

#ifdef Q_OS_LINUX
# include "V4L2Capture.h"
# include "V4L2CaptureDecouple.h"
#endif

#ifdef WITH_DIRECTSHOW
# include "directShowCapture.h"
# include "directShowCaptureDecouple.h"
#endif

#ifdef WITH_TOPVI
# include "topViCapture.h"
#endif

#ifdef WITH_UEYE
# include "uEyeCapture.h"
#endif

#ifdef WITH_SYNCCAM
#include "syncCamerasCaptureInterface.h"
#endif

#ifdef WITH_AVCODEC
#include "aviCapture.h"
#include "rtspCapture.h"
#endif

#ifdef WITH_OPENCV
 #include "opencv/openCVCapture.h"
 #include "opencv/openCVFileCapture.h"
#endif

char const *CaptureStatistics::names[] =
{
    "Desync time",
    "Internal desync",
    "Decoding time",
    "Interframe delay",
    "Frame Data size"
};

STATIC_ASSERT(CORE_COUNT_OF(CaptureStatistics::names) == CaptureStatistics::MAX_ID, wrong_comment_num_capture_stats)


ImageCaptureInterfaceQt* ImageCaptureInterfaceQtFactory::fabric(string input, bool isRGB)
{
    string file("file:");
    if (input.substr(0, file.size()).compare(file) == 0)
    {
        string tmp = input.substr(file.size());
        return new ImageCaptureInterfaceWrapper<FileCaptureInterface>(tmp);
    }

    string prec("prec:");
    if (input.substr(0, prec.size()).compare(prec) == 0)
    {
        string tmp = input.substr(prec.size());
        return new ImageCaptureInterfaceWrapper<FilePreciseCapture>(tmp, false, isRGB);
    }

#ifdef WITH_SYNCCAM
    string sync("sync:");
    if (input.substr(0, sync.size()).compare(sync) == 0)
    {
        string tmp = input.substr(sync.size());
        return new SyncCamerasCaptureInterface(tmp);
    }
#endif

#ifdef Q_OS_LINUX
    string v4l2("v4l2:");
    if (input.substr(0, v4l2.size()).compare(v4l2) == 0)
    {
        string tmp = input.substr(v4l2.size());
        return new ImageCaptureInterfaceWrapper<V4L2CaptureInterface>(tmp, isRGB);
    }

    string v4l2d("v4l2d:");
    if (input.substr(0, v4l2d.size()).compare(v4l2d) == 0)
    {
        string tmp = input.substr(v4l2d.size());
        return new ImageCaptureInterfaceWrapper<V4L2CaptureDecoupleInterface>(tmp);
    }
#endif

#ifdef WITH_TOPVI
    string topvi("topvi:");
    if (input.substr(0, topvi.size()).compare(topvi) == 0)
    {
        string tmp = input.substr(topvi.size());
        return new ImageCaptureInterfaceWrapper<TopViCaptureInterface>(tmp);
    }
#endif

#ifdef WITH_UEYE
    string ueye("ueye:");
    if (input.substr(0, ueye.size()).compare(ueye) == 0)
    {
        string tmp = input.substr(ueye.size());
        return new ImageCaptureInterfaceWrapper<UEyeCaptureInterface>(tmp);
    }
#endif

#ifdef WITH_DIRECTSHOW
    string dshow("dshow:");
    if (input.substr(0, dshow.size()).compare(dshow) == 0)
    {
        string tmp = input.substr(dshow.size());
        return new ImageCaptureInterfaceWrapper<DirectShowCaptureInterface>(tmp, isRGB);
    }

    string dshowd("dshowd:");
    if (input.substr(0, dshowd.size()).compare(dshowd) == 0)
    {
        string tmp = input.substr(dshowd.size());
        return new ImageCaptureInterfaceWrapper<DirectShowCaptureDecoupleInterface>(tmp);
    }
#endif

#ifdef WITH_AVCODEC
    string avcodec("avcodec:");
    if (input.substr(0, avcodec.size()).compare(avcodec) == 0)
    {
        SYNC_PRINT(("ImageCaptureInterface::fabric(): Creating avcodec input"));
        string tmp = input.substr(avcodec.size());
        return new ImageCaptureInterfaceWrapper<AviCapture>(QString(tmp.c_str()));
    }
#if 0
    string rtsp("rtsp:");
    if (input.substr(0, rtsp.size()).compare(rtsp) == 0)
    {
        SYNC_PRINT(("ImageCaptureInterface::fabric(): Creating avcodec input"));
        return new ImageCaptureInterfaceWrapper<RTSPCapture>(QString(input.c_str()));
    }
#endif
#endif

#ifdef WITH_OPENCV
    string any("any:");
    if (input.substr(0, any.size()).compare(any) == 0)
    {
        string tmp = input.substr(any.size());
        return new ImageCaptureInterfaceWrapper<OpenCVCaptureInterface>(tmp, CAP_ANY);
    }

    string vfw("vfw:");
    if (input.substr(0, vfw.size()).compare(vfw) == 0)
    {
        string tmp = input.substr(vfw.size());
        return new ImageCaptureInterfaceWrapper<OpenCVCaptureInterface>(tmp, CAP_VFW);
    }

    string ds("ds:");
    if (input.substr(0, ds.size()).compare(ds) == 0)
    {
        string tmp = input.substr(ds.size());
        return new ImageCaptureInterfaceWrapper<OpenCVCaptureInterface>(tmp, CAP_DS);
    }

    string opencv_file("opencv_file:");
    if (input.substr(0, opencv_file.size()).compare(opencv_file) == 0)
    {
        string tmp = input.substr(opencv_file.size());
        return new ImageCaptureInterfaceWrapper<OpenCvFileCapture>(tmp);
    }
#endif

    return NULL;
}

ImageCaptureInterfaceQt *ImageCaptureInterfaceQtFactory::fabric(string input, int h, int w, int fps, bool isRgb)
{
#ifdef Q_OS_LINUX
    string v4l2("v4l2:");
    if (input.substr(0, v4l2.size()).compare(v4l2) == 0)
    {
        string tmp = input.substr(v4l2.size());
        return new ImageCaptureInterfaceWrapper<V4L2CaptureInterface>(tmp, h, w, fps, isRgb);
    }
#endif

#ifdef WITH_TOPVI
    string topvi("topvi:");
    if (input.substr(0, topvi.size()).compare(topvi) == 0)
    {
        string tmp = input.substr(topvi.size());
        return new ImageCaptureInterfaceWrapper<TopViCaptureInterface>(tmp);
    }
#endif

#ifdef WITH_UEYE
    string ueye("ueye:");
    if (input.substr(0, ueye.size()).compare(ueye) == 0)
    {
        string tmp = input.substr(ueye.size());
        return new ImageCaptureInterfaceWrapper<UEyeCaptureInterface>(tmp, h, w, fps, isRgb);
    }
#endif

#ifdef WITH_DIRECTSHOW
    string dshow("dshow:");
    if (input.substr(0, dshow.size()).compare(dshow) == 0)
    {
        string tmp = input.substr(dshow.size());
        return new ImageCaptureInterfaceWrapper<DirectShowCaptureInterface>(tmp, h, w, fps, isRgb);
    }
#endif

    return NULL;
}

void ImageCaptureInterface::notifyAboutNewFrame(frame_data_t frameData)
{
    SYNC_PRINT(("ImageCaptureInterface::notifyAboutNewFrame()\n"));
    if (imageInterfaceReceiver != NULL)
    {
        imageInterfaceReceiver->newFrameReadyCallback(frameData);
        imageInterfaceReceiver->newImageReadyCallback();
    } else {
        SYNC_PRINT(("Warning:  ImageCaptureInterface::notifyAboutNewFrame(): imageInterfaceReceiver is NULL\n"));
    }
}

ImageCaptureInterface::ImageCaptureInterface()
   : mIsRgb(false)
{
    qRegisterMetaType<frame_data_t>("frame_data_t");
    imageInterfaceReceiver = NULL;
    SYNC_PRINT(("ImageCaptureInterface::ImageCaptureInterface(): called\n"));
}

ImageCaptureInterface::~ImageCaptureInterface()
{
    return;
}

void ImageCaptureInterface::getAllCameras(vector<string> &cameras)
{
#ifdef Q_OS_WIN
# ifdef WITH_DIRECTSHOW
    vector<string> dshowcams;
    DirectShowCaptureInterface::getAllCameras(dshowcams);
    for (const string& cam : dshowcams) {
        cameras.push_back(std::string("dshow:" + cam));
    }
# endif
#endif

#ifdef WITH_UEYE
    vector<string> ueyecams;
    UEyeCaptureInterface::getAllCameras(ueyecams);
    for (string cam: ueyecams) {
        cameras.push_back(std::string("ueye:" + cam));
    }
#endif

#ifdef WITH_TOPVI
    vector<string> topvicams;
    TopViCaptureInterface::getAllCameras(topvicams);
    for (string cam: topvicams) {
        cameras.push_back(std::string("topvi:" + cam));
    }
#endif

#ifdef Q_OS_LINUX
    vector<string> v4lcams;
    V4L2CaptureInterface::getAllCameras(v4lcams);
    for (string cam: v4lcams) {
        cameras.push_back(std::string("v4l2:" + cam));
    }
#endif
}

ImageCaptureInterface::CapErrorCode ImageCaptureInterface::setCaptureProperty(int /*id*/, int /*value*/)
{
    return FAILURE;
}

ImageCaptureInterface::CapErrorCode ImageCaptureInterface::getCaptureProperty(int /*id*/, int * /*value*/)
{
    return FAILURE;
}

ImageCaptureInterface::CapErrorCode ImageCaptureInterface::getCaptureName(QString & /*value*/)
{
    return FAILURE;
}

ImageCaptureInterface::CapErrorCode ImageCaptureInterface::getFormats(int * /*num*/, CameraFormat *& /*formats*/)
{
    return FAILURE;
}

bool ImageCaptureInterface::getCurrentFormat(ImageCaptureInterface::CameraFormat & /*format*/)
{
    return false;
}

QString ImageCaptureInterface::getInterfaceName()
{
    return "";
}

ImageCaptureInterface::CapErrorCode ImageCaptureInterface::getDeviceName(int /*num*/, QString & /*name*/)
{
    return FAILURE;
}

string ImageCaptureInterface::getDeviceSerial(int /*num*/)
{
    return "";
}

ImageCaptureInterface::CapErrorCode ImageCaptureInterface::initCapture()
{
    return FAILURE;
}

ImageCaptureInterface::CapErrorCode ImageCaptureInterface::startCapture()
{
    return FAILURE;
}

ImageCaptureInterface::CapErrorCode ImageCaptureInterface::pauseCapture()
{
    return FAILURE;
}

ImageCaptureInterface::CapErrorCode ImageCaptureInterface::nextFrame()
{
    return FAILURE;
}

ImageCaptureInterface::CapErrorCode ImageCaptureInterface::queryCameraParameters(CameraParameters &parameter)
{
    Q_UNUSED(parameter)
    return ImageCaptureInterface::SUCCESS;
}

bool ImageCaptureInterface::supportPause()
{
    return false;
}
