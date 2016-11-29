#pragma once

#include "descriptorExtractorProvider.h"

namespace cv {
    namespace gpu {
        class SURF_GPU;
        class ORB_GPU;
    }

    namespace ocl {
        class SURF_OCL;
    }
}

class OpenCvGPUDescriptorExtractorWrapper : public DescriptorExtractor
{
public:
    OpenCvGPUDescriptorExtractorWrapper( cv::gpu::SURF_GPU *extractor );
    OpenCvGPUDescriptorExtractorWrapper( cv::gpu::ORB_GPU *extractor );
    OpenCvGPUDescriptorExtractorWrapper( cv::ocl::SURF_OCL* extractor );
    ~OpenCvGPUDescriptorExtractorWrapper();

    void   setProperty(const std::string &name, const double &value);
    double getProperty(const std::string &name) const;

protected:
    void computeImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descripors);

private:
    OpenCvGPUDescriptorExtractorWrapper( const OpenCvGPUDescriptorExtractorWrapper &wrapper );

    cv::gpu::SURF_GPU *extractorSURF_CUDA;
    cv::gpu::ORB_GPU *extractorORB_CUDA;
    cv::ocl::SURF_OCL *extractorSURF_OCL;
};

extern "C"
{
    void init_opencv_gpu_descriptors_provider();
}

class OpenCvGPUDescriptorExtractorProvider : public DescriptorExtractorProviderImpl
{
public:
    OpenCvGPUDescriptorExtractorProvider( bool cudaApi );
    DescriptorExtractor* getDescriptorExtractor(const DescriptorType &type, const std::string &params = "");
    bool provides(const DescriptorType &type);

    ~OpenCvGPUDescriptorExtractorProvider() {}

    bool cudaApi;
};
