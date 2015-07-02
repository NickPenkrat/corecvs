#include "openCvDescriptorExtractorWrapper.h"
#include "openCvKeyPointsWrapper.h"

OpenCvDescriptorExtractorWrapper::OpenCvDescriptorExtractorWrapper(cv::DescriptorExtractor *extractor) : extractor(extractor) {

}

#if 0
OpenCvDescriptorExtractorWrapper::OpenCvDescriptorExtractorWrapper(const OpenCvDescriptorExtractorWrapper &w) {
	// FIXME: Will it blend?!
	extractor = new cv::DescriptorExtractor(*w.extractor);
}
#endif

OpenCvDescriptorExtractorWrapper::~OpenCvDescriptorExtractorWrapper() {
	delete extractor;
}

void OpenCvDescriptorExtractorWrapper::computeImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors) {
	std::vector<cv::KeyPoint> kps;
	for(auto kp: keyPoints)
		kps.push_back(convert(kp));
	cv::Mat img = convert(image), desc;

	extractor->compute(img, kps, desc);

	keyPoints.clear();
	for(auto kp: kps)
		keyPoints.push_back(convert(kp));

	descriptors = convert(desc);
}

void init_opencv_descriptors_provider() {
	DescriptorExtractorProvider::getInstance().add(new OpenCvDescriptorExtractorProvider());
}

#define SWITCH_TYPE(str, expr) \
	if(type == #str) { \
		expr \
	}

DescriptorExtractor* OpenCvDescriptorExtractorProvider::getDescriptorExtractor(const DescriptorType &type, const DetectorsParams &params) {
	auto siftParams = params.siftParams;
	auto surfParams = params.surfParams;
	auto briskParams = params.briskParams;
	auto orbParams = params.orbParams;
	SWITCH_TYPE(SIFT, 
		return new OpenCvDescriptorExtractorWrapper(new cv::SIFT(0, siftParams.nOctaveLayers, siftParams.contrastThreshold, siftParams.edgeThreshold, siftParams.sigma));)
	SWITCH_TYPE(SURF,
		return new OpenCvDescriptorExtractorWrapper(new cv::SURF(surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, surfParams.upright));)
	SWITCH_TYPE(BRISK,
		return new OpenCvDescriptorExtractorWrapper(new cv::BRISK(briskParams.thresh, briskParams.octaves, briskParams.patternScale));)
	SWITCH_TYPE(ORB,
		return new OpenCvDescriptorExtractorWrapper(new cv::ORB(2000, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize));)
	return 0;
}

bool OpenCvDescriptorExtractorProvider::provides(const DescriptorType &type) {
	SWITCH_TYPE(SIFT, return true;);
	SWITCH_TYPE(SURF, return true;);
	SWITCH_TYPE(BRISK, return true;);
	SWITCH_TYPE(ORB, return true;);
	return false;
}

#undef SWITCH_TYPE

