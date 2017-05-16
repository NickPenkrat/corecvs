#pragma once

#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <ostream>

#include <QImage>

#include "global.h"

#include "imageCaptureInterface.h"

#include "tpv.h"

using namespace std;

class BufferDescriptorType
{
public:

    uint8_t *buffer;
    int imageID;
    int bufferSize;

    /* Some metainfo about particular frame */

    uint64_t internalTimestamp;

    int buffersInCamera;

    uint64_t usecsTimeStamp()
    {

        time_t rawtime;
        time ( &rawtime );

        struct tm *timeinfo = localtime ( &rawtime );

        /*printf("(Curr: %d, %d %d %d:%d:%d) =>",
               timeinfo->tm_year,
               timeinfo->tm_mon,
               timeinfo->tm_mday,
               timeinfo->tm_hour,
               timeinfo->tm_min,
               timeinfo->tm_sec);*/

        uint64_t secTimeStamp = mktime ( timeinfo );

        /*printf("%" PRIu64 "\n", secTimeStamp);*/

        if (secTimeStamp == (uint64_t)(-1))
            return 0;
        return secTimeStamp * 1000 * 1000;
    }

    BufferDescriptorType()
    {
        memset(this, 0, sizeof(BufferDescriptorType));
    }
};

class TopViCameraDescriptor
{

public:
    bool inited = false;
    int  camId = -1;       //actual
    int  previewMode;  //actual
    int  toSkip = 0;

    TopViCameraDescriptor() :
        inited(false),
        camId(-1),
        previewMode(0),
        toSkip(0)
    {
    }

    ~TopViCameraDescriptor();

    struct TopVi_SK *skCamera;

    ImageCaptureInterface::CameraFormat format;

    static const unsigned   IMAGE_BUFFER_COUNT = 3;
    BufferDescriptorType    images[IMAGE_BUFFER_COUNT];

    int init(int camId, double global, double exposure);

    int fAutoExp;
    double autoExpCoef;
    double autoGlobalCoef;
    int fAutoWB;
    double autoRedCoef;
    double autoBlueCoef;

    double getExposure();
    void setExposure(double exposure);

    double getGlobalGain();
    void setGlobalGain(double global);
    double getRedGain();
    void setRedGain(double red);
    double getBlueGain();
    void setBlueGain(double blue);

    int getFrameSize(int &width, int &height) ;

    int grabFrame();

    int initBuffer();

    string getSysId();

    int replyCallback(string reply);

    uint16_t* getFrame();
    BufferDescriptorType* getDescriptorByAddress(char* pbuf);
};
