#ifdef WITH_OPENCV

#pragma once

#include "detectAndExtractProvider.h"

namespace cv {
	namespace gpu {
		class SURF_GPU;
		class ORB_GPU;
	}

	namespace ocl {
		class SURF_OCL;
	}
}

class OpenCvGPUDetectAndExtractWrapper : public DetectAndExtract
{
public:

    OpenCvGPUDetectAndExtractWrapper( cv::gpu::SURF_GPU *detector );
    OpenCvGPUDetectAndExtractWrapper( cv::gpu::ORB_GPU *detector );
    OpenCvGPUDetectAndExtractWrapper( cv::ocl::SURF_OCL *detector );

    ~OpenCvGPUDetectAndExtractWrapper();

	double getProperty(const std::string &name) const;
	void   setProperty(const std::string &name, const double &value);

	bool  isParallelable() { return false; }

protected:
    void detectAndExtractImpl( RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors, int nMaxKeypoints );

private:
    OpenCvGPUDetectAndExtractWrapper( const OpenCvGPUDetectAndExtractWrapper& );
    OpenCvGPUDetectAndExtractWrapper& operator=( const OpenCvGPUDetectAndExtractWrapper& );

	cv::gpu::SURF_GPU* detectorSURF_CUDA;
	cv::gpu::ORB_GPU*  detectorORB_CUDA;
	cv::ocl::SURF_OCL* detectorSURF_OCL;
};

extern "C"
{
    bool init_opencv_gpu_detect_and_extract_provider( bool& cudaApi );
}

class OpenCvGPUDetectAndExtractProvider : public DetectAndExtractProviderImpl
{
public:
    OpenCvGPUDetectAndExtractProvider( bool cudaApi );

    ~OpenCvGPUDetectAndExtractProvider() {}

    DetectAndExtract* getDetector( const DetectorType &detectorType, const DescriptorType &descriptorType, const std::string &params = "" );
	bool provides(const DetectorType &detectorType, const DescriptorType &descriptorType);

private:
	bool               cudaApi; // opencv module to use : cuda or opencl
};


#endif
