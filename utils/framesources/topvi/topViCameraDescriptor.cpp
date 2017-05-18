#include <iostream>
#include "topViCameraDescriptor.h"

double TopViCameraDescriptor::getMinExposure() {
    return 2.;
}

double TopViCameraDescriptor::getMaxExposure() {
    return 4000.;
}

double TopViCameraDescriptor::getDefaultExposure() {
    return 100; //ms
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

double TopViCameraDescriptor::getExposure() {
    if (fAutoExp) {
        return this->skCamera.param.exposure * this->autoExpCoef;
    }
    else {
        return this->skCamera.param.exposure;
    }
}

int TopViCameraDescriptor::setStatus(string reply) {
    if (inited) {
        int pos = reply.find_last_of(':');
        string res = reply.substr(reply.find_last_of(':') + 1);
        this->skCamera.param.exposure = QString(res.c_str()).toDouble(); //stod(res);
        reply = reply.substr(0, pos);
        res = reply.substr(reply.find_last_of(':') + 1);
        this->skCamera.param.globalGain = QString(res.c_str()).toDouble(); //stod(res);
        SYNC_PRINT((" TopViCameraDescriptor: set status to camera %d: exp = %.2lf, gain = %.2lf\n", this->camId, skCamera.param.exposure, skCamera.param.globalGain));
    }
    else {
        SYNC_PRINT((" TopViCameraDescriptor: camera is not inited\n"));
    }
    return true;
}

int TopViCameraDescriptor::setExposure(string reply) {
    string res = reply.substr(reply.find_last_of(':') + 1);
    this->skCamera.param.exposure = QString(res.c_str()).toDouble(); //stod(res);
    SYNC_PRINT((" TopViCameraDescriptor: set exposure %.2lf to camera %d\n", skCamera.param.exposure, this->camId));
    return true;
}

double TopViCameraDescriptor::getGlobalGain() {
    if (fAutoExp || fAutoWB) {
        return this->skCamera.param.globalGain * this->autoGlobalCoef;
    }
    else {
        return this->skCamera.param.globalGain;
    }
}

int TopViCameraDescriptor::setGlobalGain(string reply){
    string res = reply.substr(reply.find_last_of(':') + 1);
    this->skCamera.param.globalGain = QString(res.c_str()).toDouble(); //stod(res);
//    this->skCamera.param.globalGain = tpvGetGainValue(stod(res));
    SYNC_PRINT((" TopViCameraDescriptor: set global gain %.2lf to camera %d\n", skCamera.param.globalGain, this->camId));
    return true;
}

double TopViCameraDescriptor::getRedGain() {
    if (fAutoWB) {
        return this->skCamera.param.redGain * this->autoRedCoef;
    }
    else {
        return this->skCamera.param.redGain;
    }
}

int TopViCameraDescriptor::setRedGain(string reply) {
    string res = reply.substr(reply.find_last_of(':') + 1);
    this->skCamera.param.redGain = QString(res.c_str()).toDouble(); //stod(res);
//    this->skCamera.param.redGain = tpvGetGainValue(stod(res));
    SYNC_PRINT((" TopViCameraDescriptor: set red gain %.2lf to camera %d\n", skCamera.param.redGain, this->camId));
    return true;
}

double TopViCameraDescriptor::getBlueGain() {
    if (fAutoWB) {
        return this->skCamera.param.blueGain * this->autoBlueCoef;
    }
    else {
        return this->skCamera.param.blueGain;
    }
}

int TopViCameraDescriptor::setBlueGain(string reply) {
    string res = reply.substr(reply.find_last_of(':') + 1);
//    this->skCamera.param.blueGain = tpvGetGainValue(stod(res));
    this->skCamera.param.blueGain = QString(res.c_str()).toDouble(); //stod(res);
    SYNC_PRINT((" TopViCameraDescriptor: set blue gain %.2lf to camera %d\n", skCamera.param.blueGain, this->camId));
    return true;
}

int TopViCameraDescriptor::getFrameSize(int &width, int &height){
    switch (this->skCamera.param.size) {
        case TPV_PK_3840x2780:
        default:
            width = 3840;
            height = 2780;
    }
    format.height = height;
    format.width = width;
    format.fps = 6;
    return this->skCamera.param.size;
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

int TopViCameraDescriptor::replyCallback(enum TopViCmdName cmdName, string reply)
{
    SYNC_PRINT(("TopViCameraDescriptor replyCallback for %s: %s\n", tpvCmdName(cmdName), reply.c_str()));
    int result = false;
    switch (cmdName) {
      case TPV_EXPOSURE:
          result |= setExposure(reply);
          break;
      case TPV_GAIN:
          result |= setGlobalGain(reply);
          break;
      case TPV_STATUS:
          result |= setStatus(reply);
          break;
      default:
          SYNC_PRINT(("TopViCameraDescriptor replyCallback: do nothing, result is OK for %s\n", tpvCmdName(cmdName)));
          result = true;
    }
    return result;
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
