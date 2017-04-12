#pragma once

#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <ostream>

#include <QImage>

#include "global.h"

#include "topViCameraDescriptor.h"
#include "imageCaptureInterface.h"

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
    int  toSkip = 0;

    int  previewMode; //actual

    TopViCameraDescriptor() :
        inited(false),
        camId(-1),
        previewMode(false),
        toSkip(0)
    {
    }

    static const unsigned   IMAGE_BUFFER_COUNT = 3;
    BufferDescriptorType    images[IMAGE_BUFFER_COUNT];

    //  <sk> structure
    enum SK_STATE {
        SK_UNKNOWN = -1,
        SK_OK,
        SK_ERROR
    };

    SK_STATE statusCode = SK_UNKNOWN;

    // <pk> structure
    enum ACTIVE_MODE {
        PK_UNKNOWN = -1,
        PK_SLEEP,
        PK_LIVE,
        PK_GRAB
    };

    enum BINNING_MODE {
        PK_1X1,
        PK_2X2,
        PK_4X4
    };

    struct TopViCameraParam {
        ACTIVE_MODE activeMode = PK_UNKNOWN;
        ImageCaptureInterface::CameraFormat format;
        BINNING_MODE binningMode = PK_1X1;
        int exposure = 100000; //us
    } currentParams;

    int init(int cameraId);
    int grabFrame();

    int initBuffer();

    string getSysId();

    int replyCallback(string reply);

    uint16_t* getFrame();
    BufferDescriptorType* getDescriptorByAddress(char* pbuf);
};
