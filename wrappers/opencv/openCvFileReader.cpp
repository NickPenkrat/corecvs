#include "openCvFileReader.h"
#include "openCvKeyPointsWrapper.h"

#include <exception>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

bool OpenCvBufferReaderProvider::provides(const std::string &filename)
{
    // TODO: make list of supported formats
    return true;
}

BufferReader* OpenCvBufferReaderProvider::getBufferReader(const std::string &filename)
{
    return new OpenCvBufferReader();
}

void init_opencv_reader_provider()
{
    BufferReaderProvider::getInstance().add(new OpenCvBufferReaderProvider);
}

RuntimeTypeBuffer OpenCvBufferReader::read(const std::string &s)
{
    cv::Mat img = cv::imread(s, CV_LOAD_IMAGE_GRAYSCALE);
    if(!(img.rows && img.cols && img.data))
    {
        std::stringstream ss;
        ss << "\"" << s << "\" is not a valid image file";
        throw std::invalid_argument(ss.str());
    }
    return convert(img);
}



