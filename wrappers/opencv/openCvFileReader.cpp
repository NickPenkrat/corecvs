#include "openCvFileReader.h"
#include "openCvKeyPointsWrapper.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

bool OpenCvBufferReaderProvider::provides(const std::string &filename) {
	// TODO: make list of supported formats
	return true;
}

BufferReader* OpenCvBufferReaderProvider::getBufferReader(const std::string &filename) {
	return new OpenCvBufferReader();
}

void init_opencv_reader_provider() {
	BufferReaderProvider::getInstance().add(new OpenCvBufferReaderProvider);
}

DescriptorBuffer OpenCvBufferReader::read(const std::string &s) {
	cv::Mat img = cv::imread(s, CV_LOAD_IMAGE_GRAYSCALE);
	return convert(img);
}



