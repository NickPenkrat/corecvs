#pragma once

#include "featureDetectorProvider.h"

namespace cv {
    namespace gpu {
        class SURF_GPU;
        class ORB_GPU;
    } 

    namespace ocl {
        class SURF_OCL;
    }
}

class OpenCvGPUFeatureDetectorWrapper : public FeatureDetector
{
public:
    OpenCvGPUFeatureDetectorWrapper( cv::gpu::SURF_GPU *detector );
    OpenCvGPUFeatureDetectorWrapper( cv::gpu::ORB_GPU *detector );
    OpenCvGPUFeatureDetectorWrapper( cv::ocl::SURF_OCL *detector );

    ~OpenCvGPUFeatureDetectorWrapper();

    double getProperty(const std::string &name) const;
    void   setProperty(const std::string &name, const double &value);

    bool  isParallelable() { return false; }

protected:
    void detectImpl( RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, int nKeypoints );

private:
    OpenCvGPUFeatureDetectorWrapper( const OpenCvGPUFeatureDetectorWrapper& );
    OpenCvGPUFeatureDetectorWrapper& operator=( const OpenCvGPUFeatureDetectorWrapper& );

    cv::gpu::SURF_GPU* detectorSURF_CUDA;
    cv::gpu::ORB_GPU*  detectorORB_CUDA;
    cv::ocl::SURF_OCL* detectorSURF_OCL;   
};

extern "C"
{
    void init_opencv_gpu_detectors_provider();
}

class OpenCvGPUFeatureDetectorProvider : public FeatureDetectorProviderImpl
{
public:
    OpenCvGPUFeatureDetectorProvider( bool cudaApi );

    ~OpenCvGPUFeatureDetectorProvider() {}

    FeatureDetector* getFeatureDetector(const DetectorType &type, const std::string &params = "");
    bool provides(const DetectorType &type);

private:
    bool               cudaApi; // opencv module to use : cuda or opencl
};
