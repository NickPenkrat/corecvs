#include "openCvKeyPointsWrapper.h"
#include <iostream>
#include <string>

cv::KeyPoint convert(const KeyPoint &kp) {
	return cv::KeyPoint(kp.x, kp.y, kp.size, kp.angle, kp.response, kp.octave);
}

KeyPoint convert(const cv::KeyPoint &kp) {
	return KeyPoint(kp.pt.x, kp.pt.y, kp.size, kp.angle, kp.response, kp.octave);
}

cv::DMatch convert(const RawMatch &rm) {
	return cv::DMatch(rm.featureQ, rm.featureT, rm.distance);
}

RawMatch convert(const cv::DMatch &dm, size_t imgQ, size_t imgT) {
	return RawMatch(imgQ, imgT, dm.queryIdx, dm.trainIdx, dm.distance);
}

cv::Mat convert(const DescriptorBuffer &buffer) {
#if 0
	std::cout << "DB to Mat" << std::endl;
	std::cout << buffer.getType().getCvType() << std::endl;
	std::cout << "Type " << (std::string)buffer.getType() << std::endl;
#endif
	return cv::Mat(buffer.getRows(), buffer.getCols(), buffer.getType().getCvType(), buffer.row<void>((size_t)0));
//	return (cv::Mat)buffer;
}

DescriptorBuffer convert(const cv::Mat &mat) {
#if 0
	std::cout << "Mat to DB" << std::endl;
	std::cout << (std::string)BufferType((size_t)mat.type()) << std::endl;
#endif
return DescriptorBuffer(mat.rows, mat.cols, mat.data, BufferType((size_t)mat.type()));
}

