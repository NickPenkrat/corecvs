#ifndef OPENCVDESCRIPTORMATCHERWRAPPER_H
#define OPENCVDESCRIPTORMATCHERWRAPPER_H

#include "descriptorMatcherProvider.h"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>

class OpenCvDescriptorMatcherWrapper : public DescriptorMatcher
{
public:
	OpenCvDescriptorMatcherWrapper(cv::DescriptorMatcher *detector);
	~OpenCvDescriptorMatcherWrapper();
protected:
	void knnMatchImpl(RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch>> &matches, size_t K);
private:
	OpenCvDescriptorMatcherWrapper(const OpenCvDescriptorMatcherWrapper &wrapper);
	cv::DescriptorMatcher *matcher;
};

extern "C"
{
	void init_opencv_matchers_provider();
}

class OpenCvDescriptorMatcherProvider : public DescriptorMatcherProviderImpl
{
public:
	DescriptorMatcher* getDescriptorMatcher(const DescriptorType &descriptor, const MatcherType &matcher);
	bool provides(const DescriptorType &descriptor, const MatcherType &matcher);
	~OpenCvDescriptorMatcherProvider() {}
protected:
};


#endif
