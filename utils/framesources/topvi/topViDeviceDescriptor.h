#ifndef TOPVIDEVICEDESCRIPTOR_H
#define TOPVIDEVICEDESCRIPTOR_H

#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <ostream>

#include <QImage>

#include "global.h"

#include "topViCameraDescriptor.h"

class TopViDeviceDescriptor
{
public:
    int inited;
    int deviceId;

    QList<TopViCameraDescriptor>  mCameras;
    QImage                  *result = NULL;

    TopViDeviceDescriptor() :
        inited(false),
        deviceId(-1),
        mCameras()
    {}

    int init(int deviceId);

    bool isFilled() const    { return result != NULL; }
};

#endif // TOPVIDEVICEDESCRIPTOR_H
