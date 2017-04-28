#include <iostream>
#include "topViCameraDescriptor.h"

int TopViCameraDescriptor::init(int cameraId)
{
    CORE_UNUSED(cameraId);
    return 0;
}

int TopViCameraDescriptor::initBuffer()
{
    return 0;
}

string TopViCameraDescriptor::getSysId()
{
    std::stringstream ss;
    ss << "TopVi_" << camId;
    string dev = ss.str();
    return dev;
}

int TopViCameraDescriptor::replyCallback(string reply)
{
    CORE_UNUSED(reply);
    //TODO:
    return SK_ERROR;
}

uint16_t* TopViCameraDescriptor::getFrame()
{
    return NULL;
}

BufferDescriptorType* TopViCameraDescriptor::getDescriptorByAddress (char* pbuf)
{
    CORE_UNUSED(pbuf);
    return NULL;
}
