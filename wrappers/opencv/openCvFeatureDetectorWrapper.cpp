#include "openCvFeatureDetectorWrapper.h"
#include "openCvKeyPointsWrapper.h"

OpenCvFeatureDetectorWrapper::OpenCvFeatureDetectorWrapper(cv::FeatureDetector *detector) : detector(detector) {
}

OpenCvFeatureDetectorWrapper::~OpenCvFeatureDetectorWrapper() {
	delete detector;
}

void OpenCvFeatureDetectorWrapper::detectImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints) {
	std::vector<cv::KeyPoint> kps;
	cv::Mat img = convert(image);

	detector->detect(img, kps);

	keyPoints.clear();
	for(auto kp: kps)
		keyPoints.push_back(convert(kp));
}

void init_opencv_detectors_provider() {
	FeatureDetectorProvider::getInstance().add(new OpenCvFeatureDetectorProvider());
}

#define SWITCH_TYPE(str, expr) \
	if(type == #str) { \
		expr \
	}

FeatureDetector* OpenCvFeatureDetectorProvider::getFeatureDetector(const DetectorType &type, const DetectorsParams &params) {
	auto siftParams = params.siftParams;
	auto surfParams = params.surfParams;
	auto starParams = params.starParams;
	auto fastParams = params.fastParams;
	auto briskParams = params.briskParams;
	auto orbParams = params.orbParams;
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
		return new OpenCvFeatureDetectorWrapper(new cv::ORB(2000, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize));)
	return 0;
}

bool OpenCvFeatureDetectorProvider::provides(const DetectorType &type) {
	SWITCH_TYPE(SIFT, return true;);
	SWITCH_TYPE(SURF, return true;);
	SWITCH_TYPE(STAR, return true;);
	SWITCH_TYPE(FAST, return true;);
	SWITCH_TYPE(BRISK, return true;);
	SWITCH_TYPE(ORB, return true;);
	return false;
}

#undef SWITCH_TYPE
