#include <iostream>
#include "topViCameraDescriptor.h"

TopViCameraDescriptor::~TopViCameraDescriptor() {
    delete_safe(this->skCamera);
}

int TopViCameraDescriptor::init(int camId, double global, double exposure)
{
    this->skCamera = new TopVi_SK;
    skCamera->camId = camId;
    skCamera->param.globalGain = global;
    skCamera->param.redGain = global;
    skCamera->param.blueGain = global;
    skCamera->param.exposure = exposure;
    return 0;
}

double TopViCameraDescriptor::getExposure() {
    return this->skCamera->param.exposure;
}

void TopViCameraDescriptor::setExposure(double exposure) {
    this->skCamera->param.exposure = exposure;
}

double TopViCameraDescriptor::getGlobalGain() {
    return this->skCamera->param.globalGain;
}

void TopViCameraDescriptor::setGlobalGain(double global){
    this->skCamera->param.globalGain = global;
}

double TopViCameraDescriptor::getRedGain() {
    return this->skCamera->param.redGain;
}

void TopViCameraDescriptor::setRedGain(double red) {
    this->skCamera->param.redGain = red;
}

double TopViCameraDescriptor::getBlueGain() {
    return this->skCamera->param.blueGain;
}

void TopViCameraDescriptor::setBlueGain(double blue) {
    this->skCamera->param.blueGain = blue;
}

int TopViCameraDescriptor::getFrameSize(int &width, int &height){
    switch (this->skCamera->param.size) {
        case TPV_PK_3840x2780:
        default:
            width = 3840;
            height = 2780;
    }
    format.height = height;
    format.width = width;
    format.fps = 6;
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
    return TPV_SK_ERROR;
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
