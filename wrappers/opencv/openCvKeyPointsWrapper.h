#ifndef OPENCVKEYPOINTSWRAPPER_H
#define OPENCVKEYPOINTSWRAPPER_H

#include "imageKeyPoints.h"
#include "imageMatches.h"
#include <opencv2/features2d/features2d.hpp>

cv::KeyPoint convert(const KeyPoint &kp);

KeyPoint convert(const cv::KeyPoint &kp);

cv::DMatch convert(const RawMatch &rm);

RawMatch convert(const cv::DMatch &dm, size_t imgQ, size_t imgT);

cv::Mat convert(const DescriptorBuffer &buffer);

DescriptorBuffer convert(const cv::Mat &mat);



#endif
