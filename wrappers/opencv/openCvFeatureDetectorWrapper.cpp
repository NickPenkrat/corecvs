#include "openCvFeatureDetectorWrapper.h"
#include "openCvKeyPointsWrapper.h"

#include "openCvDefaultParams.h"

#include "global.h"

#include <opencv2/features2d/features2d.hpp>    // cv::FeatureDetector
#include <opencv2/nonfree/features2d.hpp>       // cv::SURF, cv::SIFT

using namespace corecvs;

OpenCvFeatureDetectorWrapper::OpenCvFeatureDetectorWrapper(cv::FeatureDetector *detector) : detector(detector)
{}

OpenCvFeatureDetectorWrapper::~OpenCvFeatureDetectorWrapper()
{
	delete detector;
}

double OpenCvFeatureDetectorWrapper::getProperty(const std::string &name) const
{
	return detector->get<double>(name);
}

void OpenCvFeatureDetectorWrapper::setProperty(const std::string &name, const double &value)
{
	detector->set(name, value);
}

void OpenCvFeatureDetectorWrapper::detectImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, int nKeyPoints)
{
	std::vector<cv::KeyPoint> kps;
	cv::Mat img = convert(image);

	detector->detect(img, kps);

	keyPoints.clear();

    FOREACH(const cv::KeyPoint &kp, kps)
    {
		keyPoints.push_back(convert(kp));
    }
	std::sort(keyPoints.begin(), keyPoints.end(), [](const KeyPoint &l, const KeyPoint &r) { return l.response > r.response; });
	keyPoints.resize(std::min((int)keyPoints.size(), nKeyPoints));
}

void init_opencv_detectors_provider()
{
	FeatureDetectorProvider::getInstance().add(new OpenCvFeatureDetectorProvider());
}

#define SWITCH_TYPE(str, expr) \
	if(type == #str) \
	{ \
		expr; \
	}

FeatureDetector* OpenCvFeatureDetectorProvider::getFeatureDetector(const DetectorType &type, const std::string &params)
{
	SiftParams siftParams(params);
	SurfParams surfParams(params);
	StarParams starParams(params);
	FastParams fastParams(params);
	BriskParams briskParams(params);
	OrbParams orbParams(params);
	SWITCH_TYPE(SIFT,
			return new OpenCvFeatureDetectorWrapper(new cv::SIFT(0, siftParams.nOctaveLayers, siftParams.contrastThreshold, siftParams.edgeThreshold, siftParams.sigma));)
	SWITCH_TYPE(SURF,
			return new OpenCvFeatureDetectorWrapper(new cv::SURF(surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, surfParams.upright));)
	SWITCH_TYPE(STAR,
			return new OpenCvFeatureDetectorWrapper(new cv::StarFeatureDetector(starParams.maxSize, starParams.responseThreshold, starParams.lineThresholdProjected, starParams.lineThresholdBinarized,	starParams.supressNonmaxSize));)
	SWITCH_TYPE(FAST,
			return new OpenCvFeatureDetectorWrapper(new cv::FastFeatureDetector(fastParams.threshold, fastParams.nonmaxSuppression));)
	SWITCH_TYPE(BRISK,
			return new OpenCvFeatureDetectorWrapper(new cv::BRISK(briskParams.thresh, briskParams.octaves, briskParams.patternScale));)
	SWITCH_TYPE(ORB,
            return new OpenCvFeatureDetectorWrapper(new cv::ORB(orbParams.maxFeatures, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize));)
	return 0;
}

bool OpenCvFeatureDetectorProvider::provides(const DetectorType &type)
{
	SWITCH_TYPE(SIFT, return true;);
	SWITCH_TYPE(SURF, return true;);
	SWITCH_TYPE(STAR, return true;);
	SWITCH_TYPE(FAST, return true;);
	SWITCH_TYPE(BRISK, return true;);
	SWITCH_TYPE(ORB, return true;);
	return false;
}

#undef SWITCH_TYPE
