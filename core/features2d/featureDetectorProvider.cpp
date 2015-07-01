#include "featureDetectorProvider.h"


#include <cassert>

#if 0
cv::FeatureDetector* FeatureDetectorProvider::getFeatureDetector(const DetectorType &type, const DetectorsParams &params) {
	auto siftParams = params.siftParams;
	auto surfParams = params.surfParams;
	auto starParams = params.starParams;
	auto fastParams = params.fastParams;
	auto briskParams = params.briskParams;
	auto orbParams = params.orbParams;

	switch(type) {
		case FeatureDetectorType::SIFT:
			return new cv::SIFT(0, siftParams.nOctaveLayers, siftParams.contrastThreshold, siftParams.edgeThreshold,
					siftParams.sigma);
		case FeatureDetectorType::SURF:
			return new cv::SURF(surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, 
					surfParams.extended, surfParams.upright);
		case FeatureDetectorType::STAR:
			return new cv::StarFeatureDetector(starParams.maxSize, starParams.responseThreshold,
					starParams.lineThresholdProjected, starParams.lineThresholdBinarized,
					starParams.supressNonmaxSize);
		case FeatureDetectorType::FAST:
			return new cv::FastFeatureDetector(fastParams.threshold, fastParams.nonmaxSuppression);
		case FeatureDetectorType::BRISK:
			return new cv::BRISK(briskParams.thresh, briskParams.octaves, briskParams.patternScale);
		case FeatureDetectorType::ORB:
			return new cv::ORB(2000, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold,
					orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize);
		case FeatureDetectorType::SIFTGPU:
			return new SiftGpu();
		default:
			return 0;
	}
}

FeatureDetectorProvider::FeatureDetectorProvider() {
}
#endif



FeatureDetector* FeatureDetectorProvider::getDetector(const DetectorType &type, const DetectorsParams &params) {
	for(auto p: providers) {
		if(p->provides(type)) {
			return p->getFeatureDetector(type, params);
		}
	}
	assert(false);
	return 0;
}

void FeatureDetector::detect(DescriptorBuffer &image, std::vector<KeyPoint> &keyPoints) {
	detectImpl(image, keyPoints);
}

FeatureDetectorProvider::~FeatureDetectorProvider() {
	for(auto p: providers)
		delete p;
	providers.clear();
}

void FeatureDetectorProvider::add(FeatureDetectorProviderImpl *provider) {
	providers.push_back(provider);
}

FeatureDetectorProvider& FeatureDetectorProvider::getInstance() {
	static FeatureDetectorProvider provider;
	return provider;
}

FeatureDetectorProvider::FeatureDetectorProvider() {
}
