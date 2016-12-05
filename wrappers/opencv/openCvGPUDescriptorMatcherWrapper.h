#pragma once

#include "descriptorMatcherProvider.h"

#ifndef WITH_OPENCV_3x

namespace cv {
    namespace gpu {
        class BruteForceMatcher_GPU_base;
    }

    namespace ocl {
        class BruteForceMatcher_OCL_base;
    }
}

class OpenCvGPUDescriptorMatcherWrapper : public DescriptorMatcher
{
public:
    OpenCvGPUDescriptorMatcherWrapper( cv::gpu::BruteForceMatcher_GPU_base *matcher );
    OpenCvGPUDescriptorMatcherWrapper( cv::ocl::BruteForceMatcher_OCL_base *matcher );
    ~OpenCvGPUDescriptorMatcherWrapper();

    void   setProperty(const std::string &name, const double &value);
    double getProperty(const std::string &name) const;

protected:
    void knnMatchImpl(RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch> > &matches, size_t K);

private:
    OpenCvGPUDescriptorMatcherWrapper( const OpenCvGPUDescriptorMatcherWrapper &wrapper );

    cv::gpu::BruteForceMatcher_GPU_base *matcherBF_CUDA;
    cv::ocl::BruteForceMatcher_OCL_base *matcherBF_OCL;
};

class OpenCvGPUDescriptorMatcherProvider : public DescriptorMatcherProviderImpl
{
public:
    OpenCvGPUDescriptorMatcherProvider( bool cudaApi );

    DescriptorMatcher* getDescriptorMatcher(const DescriptorType &descriptor, const MatcherType &matcher, const std::string &params = "");
    bool provides(const DescriptorType &descriptor, const MatcherType &matcher);

    ~OpenCvGPUDescriptorMatcherProvider() {}

    bool cudaApi;
};

#endif

extern "C"
{
    void init_opencv_gpu_matchers_provider();
}
