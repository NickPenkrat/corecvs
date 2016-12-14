#include "openCvGPUDetectAndExtractWrapper.h"
#include "openCvKeyPointsWrapper.h"
#include "global.h"
#include "bufferReaderProvider.h"
#include "featureMatchingPipeline.h"
#include "openCvDefaultParams.h"

#ifdef WITH_OPENCV_GPU

#ifndef WITH_OPENCV_3x

#include <opencv2/nonfree/gpu.hpp>       // cv::gpu::SURF
#include <opencv2/nonfree/ocl.hpp>       // cv::ocl::SURF
#include <opencv2/gpu/gpu.hpp>           // cv::gpu::ORB
// #include <opencv2/ocl/ocl.hpp>        // cv::ocl::ORB - not implemented

using namespace cv::gpu;
using namespace cv::ocl;

template < typename T >
T downsample( const T& original, float factor )
{
    T downsampled;
    resize( original, downsampled, cv::Size(), factor, factor, cv::INTER_LINEAR );
    return downsampled;
}

void OpenCvGPUDetectAndExtractWrapper::detectAndExtractImpl( RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors, int nMaxKeypoints )
{
    if ( image.getType() != BufferType::U8 || !image.isValid() )
    {
        std::cerr << __LINE__ << "Invalid image type" << std::endl;
    }

    const bool resize = false;
    float uniformScaleFactor = 1.0f;

    std::vector<cv::KeyPoint> kps;
    cv::Mat cv_descriptors;
    if ( detectorSURF_CUDA || detectorORB_CUDA )
    {
        GpuMat img( convert( image ) );
        if ( resize )
            img = downsample( img, uniformScaleFactor );

        GpuMat cudaDescriptors;
        GpuMat mask;
        if ( detectorSURF_CUDA )
        {
            //max keypoints = min(keypointsRatio * img.size().area(), 65535)
            detectorSURF_CUDA->keypointsRatio = ( float )nMaxKeypoints / img.size().area();
            ( *detectorSURF_CUDA )( img, mask, kps, cudaDescriptors, false );
        }
        else if ( detectorORB_CUDA )
        {
            ( *detectorORB_CUDA )( img, mask, kps, cudaDescriptors );
        }

        cudaDescriptors.download( cv_descriptors ); // copy GPU -> CPU

    }
    else if ( detectorSURF_OCL )
    {
        oclMat img( convert( image ) );
        if ( resize )
            img = downsample( img, uniformScaleFactor );

        oclMat mask;
        oclMat oclDescriptors;
        //max keypoints = min(keypointsRatio * img.size().area(), 65535)
        detectorSURF_OCL->keypointsRatio = ( float )nMaxKeypoints / img.size().area();
        ( *detectorSURF_OCL )( img, mask, kps, oclDescriptors, false );
        oclDescriptors.download( cv_descriptors ); // copy GPU -> CPU
    }

    keyPoints.clear();
    FOREACH( const cv::KeyPoint &kp, kps )
    {
        keyPoints.push_back( convert( kp ) );
    }

    descriptors = convert( cv_descriptors );
}

OpenCvGPUDetectAndExtractWrapper::OpenCvGPUDetectAndExtractWrapper( SURF_GPU *detector ) : detectorSURF_CUDA( detector ),
detectorORB_CUDA( 0 ),
detectorSURF_OCL( 0 )
{}

OpenCvGPUDetectAndExtractWrapper::OpenCvGPUDetectAndExtractWrapper( ORB_GPU *detector ) : detectorORB_CUDA( detector ),
detectorSURF_CUDA( 0 ),
detectorSURF_OCL( 0 )
{}

OpenCvGPUDetectAndExtractWrapper::OpenCvGPUDetectAndExtractWrapper( SURF_OCL *detector ) : detectorSURF_OCL( detector ),
detectorSURF_CUDA( 0 ),
detectorORB_CUDA( 0 )
{}

OpenCvGPUDetectAndExtractWrapper::~OpenCvGPUDetectAndExtractWrapper()
{
    delete detectorORB_CUDA;
    delete detectorSURF_OCL;
    delete detectorSURF_CUDA;
}

extern bool FindGPUDevice(bool& cudaApi);

bool  init_opencv_gpu_detect_and_extract_provider( bool& cudaApi )
{
	cudaApi = false;
    if ( FindGPUDevice( cudaApi ) )
    {
        DetectAndExtractProvider::getInstance().add( new OpenCvGPUDetectAndExtractProvider( cudaApi ) );
        return true;
    }
		
    return false;
}

OpenCvGPUDetectAndExtractProvider::OpenCvGPUDetectAndExtractProvider( bool cudaApi ) : cudaApi( cudaApi ) {}

#define SWITCH_TYPE(str, expr) \
	if(type == #str) \
		{ \
		expr; \
		}

DetectAndExtract* OpenCvGPUDetectAndExtractProvider::getDetector( const DetectorType &detectorType, const DescriptorType &descriptorType, const std::string &params )
{
	SurfParams surfParams(params);
	OrbParams orbParams(params);

    if ( cudaApi )
    {
        if ( detectorType == "SURF_GPU" && descriptorType == "SURF_GPU" )
            return new OpenCvGPUDetectAndExtractWrapper( new cv::gpu::SURF_GPU( surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, 0.01f, surfParams.upright ) );

        if ( detectorType == "ORB_GPU" && descriptorType == "ORB_GPU" )
            return new OpenCvGPUDetectAndExtractWrapper( new cv::gpu::ORB_GPU( orbParams.maxFeatures, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize ) );
    }
    else if ( detectorType == "SURF_GPU" && descriptorType == "SURF_GPU" )
        return new OpenCvGPUDetectAndExtractWrapper( new cv::ocl::SURF_OCL( surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, 0.01f, surfParams.upright ) );
	
    return 0;
}

bool OpenCvGPUDetectAndExtractProvider::provides( const DetectorType &detectorType, const DescriptorType &descriptorType )
{
    if ( cudaApi && detectorType == "ORB_GPU" && descriptorType == "ORB_GPU" )
        return true;

    if ( detectorType == "SURF_GPU" && descriptorType == "SURF_GPU" )
        return true;

	return false;
}

#else // def WITH_OPENCV_3x
bool  init_opencv_gpu_detect_and_extract_provider( bool& cudaApi )
{
	cudaApi = false;
	return false;
}
#endif

#endif