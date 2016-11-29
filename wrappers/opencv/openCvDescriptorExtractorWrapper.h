#pragma once

#include "descriptorExtractorProvider.h"

namespace cv {
#ifdef WITH_OPENCV_3x
	class Feature2D;
	typedef Feature2D DescriptorExtractor;
#else
	class DescriptorExtractor;
#endif  
}

#ifdef WITH_OPENCV_3x
struct SmartPtrHolder;
#endif  

class OpenCvDescriptorExtractorWrapper : public DescriptorExtractor
{
public:
#ifdef WITH_OPENCV_3x
    OpenCvDescriptorExtractorWrapper(SmartPtrHolder *holder);
#else
    OpenCvDescriptorExtractorWrapper(cv::DescriptorExtractor *detector);
#endif
   ~OpenCvDescriptorExtractorWrapper();

    void   setProperty(const std::string &name, const double &value);
    double getProperty(const std::string &name) const;

protected:
    void computeImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descripors);

private:
    OpenCvDescriptorExtractorWrapper(const OpenCvDescriptorExtractorWrapper &wrapper);

    cv::DescriptorExtractor *extractor;

#ifdef WITH_OPENCV_3x
    SmartPtrHolder* holder;
#endif
};

extern "C"
{
    void init_opencv_descriptors_provider();
}

class OpenCvDescriptorExtractorProvider : public DescriptorExtractorProviderImpl
{
public:
    DescriptorExtractor* getDescriptorExtractor(const DescriptorType &type, const std::string &params = "");
    bool provides(const DescriptorType &type);

    ~OpenCvDescriptorExtractorProvider() {}
};
