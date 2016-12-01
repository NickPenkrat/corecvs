#pragma once

#include "descriptorExtractorProvider.h"

namespace cv {
    class DescriptorExtractor;
}

class OpenCvDescriptorExtractorWrapper : public DescriptorExtractor
{
public:
    OpenCvDescriptorExtractorWrapper(cv::DescriptorExtractor *detector);
   ~OpenCvDescriptorExtractorWrapper();

    void   setProperty(const std::string &name, const double &value);
    double getProperty(const std::string &name) const;

protected:
    void computeImpl(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, corecvs::RuntimeTypeBuffer &descripors);

private:
    OpenCvDescriptorExtractorWrapper(const OpenCvDescriptorExtractorWrapper &wrapper);

    cv::DescriptorExtractor *extractor;
};

extern "C"
{
    void init_opencv_descriptors_provider();
}

class OpenCvDescriptorExtractorProvider : public DescriptorExtractorProviderImpl
{
public:
    DescriptorExtractor* getDescriptorExtractor(const DescriptorType &type, const std::string &params = "");
    virtual bool provides(const DescriptorType &type) override;
    virtual std::string name() override {return "OpenCV";}

    ~OpenCvDescriptorExtractorProvider() {}

};
