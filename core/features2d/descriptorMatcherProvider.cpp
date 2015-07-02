#include "descriptorMatcherProvider.h"

#include <cassert>

#if 0
#include "siftGpuMatcherWrapper.h"
#endif

#if 0
DescriptorMatcherProvider::DescriptorMatcherProvider() {
}

#define SWITCH_TYPE(str, expr) \
	if(type == #str) { \
		expr;\
	}

cv::DescriptorMatcher* DescriptorMatcherProvider::getDescriptorMatcher(const DescriptorType &type) {
#if 0
	switch(type) {
		case DescriptorExtractorType::SIFT:
		case DescriptorExtractorType::SURF:
			return new cv::FlannBasedMatcher;
		case DescriptorExtractorType::ORB:
		case DescriptorExtractorType::BRISK:
			return new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20, 10, 2));
		case DescriptorExtractorType::SIFTGPU:
			return new SiftGpuMatcher();
	}
#else
	SWITCH_TYPE(SIFT, return new cv::FlannBasedMatcher;);
	SWITCH_TYPE(SURF, return new cv::FlannBasedMatcher;);
	SWITCH_TYPE(ORB, return new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20, 10, 2)););
	SWITCH_TYPE(BRISK, return new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20, 10, 2)););
	SWITCH_TYPE(SIFTGPU, return new SiftGpuMatcher(););
	

#endif
	assert(false);
	return 0;
}
#endif 

DescriptorMatcher* DescriptorMatcherProvider::getMatcher(const DescriptorType &type, const DetectorsParams &params) {
	for(auto p: providers) {
		if(p->provides(type)) {
			return p->getDescriptorMatcher(type, params);
		}
	}
	assert(false);
	return 0;
}

void DescriptorMatcher::knnMatch(RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch>> &matches, size_t K) {
	knnMatchImpl(query, train, matches, K);
}

DescriptorMatcherProvider::~DescriptorMatcherProvider() {
	for(auto p: providers)
		delete p;
	providers.clear();
}

void DescriptorMatcherProvider::add(DescriptorMatcherProviderImpl *provider) {
	providers.push_back(provider);
}

DescriptorMatcherProvider& DescriptorMatcherProvider::getInstance() {
	static DescriptorMatcherProvider provider;
	return provider;
}

DescriptorMatcherProvider::DescriptorMatcherProvider() {
}
