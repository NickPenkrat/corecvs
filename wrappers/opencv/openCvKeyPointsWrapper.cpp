#include "openCvKeyPointsWrapper.h"

using namespace corecvs;

cv::KeyPoint convert(const KeyPoint &kp)
{
    return cv::KeyPoint(kp.position.x(), kp.position.y(), kp.size, kp.angle, kp.response, kp.octave);
}

KeyPoint convert(const cv::KeyPoint &kp)
{
	return KeyPoint(kp.pt.x, kp.pt.y, kp.size, kp.angle, kp.response, kp.octave);
}

cv::DMatch convert(const RawMatch &rm)
{
	return cv::DMatch(rm.featureQ, rm.featureT, rm.distance);
}

RawMatch convert(const cv::DMatch &dm)
{
	return RawMatch(dm.queryIdx, dm.trainIdx, dm.distance);
}
