#ifndef OPENCVFILEREADER_H
#define OPENCVFILEREADER_H

#include "bufferLoader.h"
#include "bufferFactory.h"
#include "bufferReaderProvider.h"


class OpenCvBufferReader : public BufferReader
{
public:
    corecvs::RuntimeTypeBuffer read(const std::string &s);
    corecvs::RGB24Buffer readRgb(const std::string &s);
    void write(const corecvs::RuntimeTypeBuffer& buffer, const std::string &s);
    void writeRgb(const corecvs::RGB24Buffer& buffer, const std::string &s);
    ~OpenCvBufferReader() {}
};

class OpenCvBufferReaderProvider : public BufferReaderProviderImpl
{
public:
    BufferReader* getBufferReader(const std::string &filename);
    bool provides(const std::string &filename);
    ~OpenCvBufferReaderProvider() {}
};

/**/
class OpenCVLoaderRGB24Loader : public corecvs::BufferLoader<corecvs::RGB24Buffer>
{
public:
    OpenCVLoaderRGB24Loader();
    virtual bool acceptsFile(std::string name) override;
    virtual corecvs::RGB24Buffer *load(std::string name) override;
    virtual std::string name() override {return "OpenCV";}
};

class OpenCVLoaderRuntimeTypeBufferLoader : public corecvs::BufferLoader<corecvs::RuntimeTypeBuffer>
{
public:
    OpenCVLoaderRuntimeTypeBufferLoader();
    virtual bool acceptsFile(std::string name) override;
    virtual corecvs::RuntimeTypeBuffer *load(std::string name) override;
    virtual std::string name() override {return "OpenCV";}
};


extern "C"
{
    void init_opencv_reader_provider();
}

#endif
