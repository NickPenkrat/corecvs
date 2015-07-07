#ifndef OPENCVDESCRIPTOREXTRACTORWRAPPER_H
#define OPENCVDESCRIPTOREXTRACTORWRAPPER_H

#include "descriptorExtractorProvider.h"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>

class OpenCvDescriptorExtractorWrapper : public DescriptorExtractor {
public:
	OpenCvDescriptorExtractorWrapper(cv::DescriptorExtractor *detector);
	~OpenCvDescriptorExtractorWrapper();
	void setProperty(const std::string &name, const double &value);
	double getProperty(const std::string &name) const;
protected:
	void computeImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descripors);
private:
	OpenCvDescriptorExtractorWrapper(const OpenCvDescriptorExtractorWrapper &wrapper);
	cv::DescriptorExtractor *extractor;
};

void __attribute__ ((constructor)) __attribute__ ((used)) init_opencv_descriptors_provider();


class OpenCvDescriptorExtractorProvider : public DescriptorExtractorProviderImpl {
	public:
		DescriptorExtractor* getDescriptorExtractor(const DescriptorType &type);
		bool provides(const DescriptorType &type);
		~OpenCvDescriptorExtractorProvider() {}
	protected:
};


#endif
