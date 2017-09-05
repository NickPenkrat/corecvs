#include "openCvFileReader.h"
#include "openCvKeyPointsWrapper.h"
#include "OpenCVTools.h"

#include <exception>
#include <sstream>

#include <opencv2/core/core.hpp>        // Mat
#include <opencv2/highgui/highgui.hpp>  // imread

using namespace corecvs;

bool OpenCvBufferReaderProvider::provides(const std::string &filename)
{
    // TODO: make list of supported formats
    CORE_UNUSED(filename);
    return true;
}

BufferReader* OpenCvBufferReaderProvider::getBufferReader(const std::string &filename)
{
    CORE_UNUSED(filename);
    return new OpenCvBufferReader();
}

void init_opencv_reader_provider()
{
    BufferReaderProvider::getInstance().add(new OpenCvBufferReaderProvider);

    BufferFactory::getInstance()->registerLoader(new OpenCVLoaderRGB24Loader());
    BufferFactory::getInstance()->registerLoader(new OpenCVLoaderRuntimeTypeBufferLoader());
}

static void throwInvalidArg(const std::string &name)
{
    std::stringstream ss;
    ss << "\"" << name << "\" is not a valid image file";
    throw std::invalid_argument(ss.str());
}

corecvs::RGB24Buffer OpenCvBufferReader::readRgb(const std::string &s)
{
    cv::Mat img = cv::imread(s, CV_LOAD_IMAGE_COLOR);
    if (!(img.rows && img.cols && img.data)) {
        throwInvalidArg(s);
    }
    IplImage ip(img);
    auto* b = OpenCVTools::getRGB24BufferFromCVImage(&ip);
    corecvs::RGB24Buffer buffer = *b;
    delete b;
    return buffer;
}

RuntimeTypeBuffer OpenCvBufferReader::read(const std::string &s)
{
    cv::Mat img = cv::imread(s, CV_LOAD_IMAGE_GRAYSCALE);
    if (!(img.rows && img.cols && img.data)) {
        throwInvalidArg(s);
    }
    return convert(img);
}

void OpenCvBufferReader::writeRgb(const corecvs::RGB24Buffer &buffer, const std::string &s)
{
    auto* b = OpenCVTools::getCVImageFromRGB24Buffer(&const_cast<corecvs::RGB24Buffer&>(buffer));
    CVMAT_FROM_IPLIMAGE( mat, b, false );
    cv::imwrite(s, mat);
    cvReleaseImage(&b);
}

void OpenCvBufferReader::write(const RuntimeTypeBuffer &buffer, const std::string &s)
{
    cv::Mat img = convert(buffer);
    cv::imwrite(s, img);
}


/* Remove copypaste here*/
OpenCVLoaderRGB24Loader::OpenCVLoaderRGB24Loader()
{}

bool OpenCVLoaderRGB24Loader::acceptsFile(std::string name)
{
    CORE_UNUSED(name);
    return true;
}

/* I don't see how an inability to load particular format is an exceptional situation */
RGB24Buffer *OpenCVLoaderRGB24Loader::load(std::string name)
{
    cv::Mat img = cv::imread(name, CV_LOAD_IMAGE_COLOR);
    if (!(img.rows && img.cols && img.data)) {
        throwInvalidArg(name);
    }
    IplImage ip(img);
    return OpenCVTools::getRGB24BufferFromCVImage(&ip);
}

OpenCVLoaderRuntimeTypeBufferLoader::OpenCVLoaderRuntimeTypeBufferLoader()
{}

bool OpenCVLoaderRuntimeTypeBufferLoader::acceptsFile(std::string name)
{
     CORE_UNUSED(name);
     return true;
}

RuntimeTypeBuffer *OpenCVLoaderRuntimeTypeBufferLoader::load(std::string name)
{
    cv::Mat img = cv::imread(name, CV_LOAD_IMAGE_GRAYSCALE);
    if (!(img.rows && img.cols && img.data)) {
        throwInvalidArg(name);
    }
    RuntimeTypeBuffer *header = new RuntimeTypeBuffer;
    *header = convert(img);
    return header;
}
