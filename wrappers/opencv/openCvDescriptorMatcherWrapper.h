#ifndef OPENCVDESCRIPTORMATCHERWRAPPER_H
#define OPENCVDESCRIPTORMATCHERWRAPPER_H

#include "descriptorMatcherProvider.h"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>

class OpenCvDescriptorMatcherWrapper : public DescriptorMatcher {
public:
	OpenCvDescriptorMatcherWrapper(cv::DescriptorMatcher *detector);
	~OpenCvDescriptorMatcherWrapper();
protected:
	void knnMatchImpl(RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch>> &matches, size_t K); 
private:
	OpenCvDescriptorMatcherWrapper(const OpenCvDescriptorMatcherWrapper &wrapper);
	cv::DescriptorMatcher *matcher;
};

void __attribute__ ((constructor)) __attribute__ ((used)) init_opencv_matchers_provider();


class OpenCvDescriptorMatcherProvider : public DescriptorMatcherProviderImpl {
	public:
		DescriptorMatcher* getDescriptorMatcher(const DescriptorType &type, const DetectorsParams &params = DetectorsParams());
		bool provides(const DescriptorType &type);
		~OpenCvDescriptorMatcherProvider() {}
	protected:
};


#endif
