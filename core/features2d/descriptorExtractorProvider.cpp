#include "descriptorExtractorProvider.h"


#if 0
cv::DescriptorExtractor* DescriptorExtractorProvider::getDescriptorExtractor(const DescriptorType &type, const DescriptorsParams &params) {
	auto siftParams = params.siftParams;
	auto surfParams = params.surfParams;
	auto briskParams = params.briskParams;
	auto orbParams = params.orbParams;

	switch(type) {
		case DescriptorExtractorType::SIFT:
			return new cv::SIFT(0, siftParams.nOctaveLayers, siftParams.contrastThreshold, siftParams.edgeThreshold,
					siftParams.sigma);
		case DescriptorExtractorType::SURF:
			return new cv::SURF(surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, 
					surfParams.extended, surfParams.upright);
		case DescriptorExtractorType::BRISK:
			return new cv::BRISK(briskParams.thresh, briskParams.octaves, briskParams.patternScale);
		case DescriptorExtractorType::ORB:
			return new cv::ORB(500, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold,
					orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize);
		case DescriptorExtractorType::SIFTGPU:
			return new SiftGpu();
		default:
			return 0;
	}
}

DescriptorExtractorProvider::DescriptorExtractorProvider() {
}
#endif

DescriptorExtractor* DescriptorExtractorProvider::getDescriptorExtractor(const DescriptorType &type, const DetectorsParams &params) {
	for(auto p: providers) {
		if(p->provides(type)) {
			return p->getDescriptorExtractor(type, params);
		}
	}
	assert(false);
	return 0;
}

void DescriptorExtractorProvider::add(DescriptorExtractorProviderImpl *provider) {
	providers.push_back(provider);
}

DescriptorExtractorProvider::~DescriptorExtractorProvider() {
	for(auto p: providers)
		delete p;
	providers.clear();
}

DescriptorExtractorProvider& DescriptorExtractorProvider::getInstance() {
	static DescriptorExtractorProvider provider;
	return provider;
}

DescriptorExtractorProvider::DescriptorExtractorProvider() {
}

void DescriptorExtractor::compute(DescriptorBuffer &image, std::vector<KeyPoint> &keyPoints, DescriptorBuffer &buffer) {
	computeImpl(image, keyPoints, buffer);
}
