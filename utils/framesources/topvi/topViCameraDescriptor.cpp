#include <iostream>
#include "topViCameraDescriptor.h"

int TopViCameraDescriptor::init(int camId, double global, double exposure)
{
    this->camId = camId;
    fAutoExp = false;
    fAutoWB = false;
    autoExpCoef = 1;
    autoGlobalCoef = 1;
    autoBlueCoef = 1;
    autoRedCoef = 1;
    skCamera->camId = camId;
    skCamera->param.globalGain = global;
    skCamera->param.redGain = global;
    skCamera->param.blueGain = global;
    skCamera->param.exposure = exposure;
    return 0;
}

double TopViCameraDescriptor::getMinExposure() {
    return 2.;
}

double TopViCameraDescriptor::getMaxExposure() {
    return 4000.;
}

double TopViCameraDescriptor::getDefaultExposure() {
    return 100; //ms
}

double TopViCameraDescriptor::getExposure() {
    if (fAutoExp) {
        return this->skCamera->param.exposure * this->autoExpCoef;
    }
    else {
        return this->skCamera->param.exposure;
    }
}

int TopViCameraDescriptor::setStatus(string reply) {
    string res = reply.substr(reply.find_last_of(':') + 1);
    this->skCamera->param.exposure = stod(res);
    SYNC_PRINT((" TopViCameraDescriptor: set status to camera %d: exp = %.2lf\n", this->camId, skCamera->param.exposure));
    return true;
}

int TopViCameraDescriptor::setExposure(string reply) {
    string res = reply.substr(reply.find_last_of(':') + 1);
    this->skCamera->param.exposure = stod(res);
    SYNC_PRINT((" TopViCameraDescriptor: set exposure %.2lf to camera %d\n", skCamera->param.exposure, this->camId));
    return true;
}

double TopViCameraDescriptor::getMinGain() {
    return 1;
}

double TopViCameraDescriptor::getMaxGain() {
    return 32;
}

double TopViCameraDescriptor::getDefaultGain() {
    return 1;
}

double TopViCameraDescriptor::getGlobalGain() {
    if (fAutoExp || fAutoWB) {
        return this->skCamera->param.globalGain * this->autoGlobalCoef;
    }
    else {
        return this->skCamera->param.globalGain;
    }
}

int TopViCameraDescriptor::setGlobalGain(string reply){
    string res = reply.substr(reply.find_last_of(':') + 1);
    this->skCamera->param.globalGain = stod(res);
    SYNC_PRINT((" TopViCameraDescriptor: set global gain %.2lf to camera %d\n", skCamera->param.globalGain, this->camId));
    return true;
}

double TopViCameraDescriptor::getRedGain() {
    if (fAutoWB) {
        return this->skCamera->param.redGain * this->autoRedCoef;
    }
    else {
        return this->skCamera->param.redGain;
    }
}

int TopViCameraDescriptor::setRedGain(string reply) {
    string res = reply.substr(reply.find_last_of(':') + 1);
    this->skCamera->param.redGain = stod(res);
    SYNC_PRINT((" TopViCameraDescriptor: set red gain %.2lf to camera %d\n", skCamera->param.redGain, this->camId));
    return true;
}

double TopViCameraDescriptor::getBlueGain() {
    if (fAutoWB) {
        return this->skCamera->param.blueGain * this->autoBlueCoef;
    }
    else {
        return this->skCamera->param.blueGain;
    }
}

int TopViCameraDescriptor::setBlueGain(string reply) {
    string res = reply.substr(reply.find_last_of(':') + 1);
    this->skCamera->param.blueGain = stod(res);
    SYNC_PRINT((" TopViCameraDescriptor: set blue gain %.2lf to camera %d\n", skCamera->param.blueGain, this->camId));
    return true;
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
