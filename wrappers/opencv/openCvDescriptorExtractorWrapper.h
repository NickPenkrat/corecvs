#ifndef OPENCVDESCRIPTOREXTRACTORWRAPPER_H
#define OPENCVDESCRIPTOREXTRACTORWRAPPER_H

#include "descriptorExtractorProvider.h"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>

class OpenCvDescriptorExtractorWrapper : public DescriptorExtractor {
public:
	OpenCvDescriptorExtractorWrapper(cv::DescriptorExtractor *detector);
	~OpenCvDescriptorExtractorWrapper();
protected:
	void computeImpl(DescriptorBuffer &image, std::vector<KeyPoint> &keyPoints, DescriptorBuffer &descripors);
private:
	OpenCvDescriptorExtractorWrapper(const OpenCvDescriptorExtractorWrapper &wrapper);
	cv::DescriptorExtractor *extractor;
};

void __attribute__ ((constructor)) __attribute__ ((used)) init_opencv_descriptors_provider();


class OpenCvDescriptorExtractorProvider : public DescriptorExtractorProviderImpl {
	public:
		DescriptorExtractor* getDescriptorExtractor(const DescriptorType &type, const DetectorsParams &params = DetectorsParams());
		bool provides(const DescriptorType &type);
		~OpenCvDescriptorExtractorProvider() {}
	protected:
};


#endif
