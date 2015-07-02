#include "featureDetectorProvider.h"


#include <cassert>

FeatureDetector* FeatureDetectorProvider::getDetector(const DetectorType &type, const DetectorsParams &params) {
	for(auto p: providers) {
		if(p->provides(type)) {
			return p->getFeatureDetector(type, params);
		}
	}
	assert(false);
	return 0;
}

void FeatureDetector::detect(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints) {
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
