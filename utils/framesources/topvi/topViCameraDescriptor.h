#ifndef TOPVICAMERADESCRIPTOR_H
#define TOPVICAMERADESCRIPTOR_H

#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <ostream>

#include <QImage>

#include "global.h"

#include "topViCameraDescriptor.h"

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
    bool inited;
    int  camId;
    int  toSkip;

    TopViCameraDescriptor() :
        inited(false),
        camId(-1),
        toSkip(0)
    {}

    static const unsigned   IMAGE_BUFFER_COUNT = 3;
    BufferDescriptorType    images[IMAGE_BUFFER_COUNT];

    int init(int cameraId);
    int grabFrame();

    int initBuffer();
    uint16_t* getFrame();
    BufferDescriptorType* getDescriptorByAddress (char* pbuf);

};

#endif // TOPVICAMERADESCRIPTOR_H
