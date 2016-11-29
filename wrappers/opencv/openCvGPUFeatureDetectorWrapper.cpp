#include "openCvGPUFeatureDetectorWrapper.h"
#include "openCvKeyPointsWrapper.h"

#include "openCvDefaultParams.h"

#include "global.h"

#include <opencv2/features2d/features2d.hpp>    // cv::FeatureDetector
#include <opencv2/nonfree/gpu.hpp>       // cv::gpu::SURF
#include <opencv2/nonfree/ocl.hpp>       // cv::ocl::SURF
#include <opencv2/gpu/gpu.hpp>           // cv::gpu::ORB
// #include <opencv2/ocl/ocl.hpp>        // cv::ocl::ORB - not implemented

using namespace cv::gpu;
using namespace cv::ocl;

OpenCvGPUFeatureDetectorWrapper::OpenCvGPUFeatureDetectorWrapper( cv::gpu::SURF_GPU *detector ) : detectorSURF_CUDA( detector ),
    detectorORB_CUDA( 0 ),
    detectorSURF_OCL( 0 )
{}

OpenCvGPUFeatureDetectorWrapper::OpenCvGPUFeatureDetectorWrapper( cv::gpu::ORB_GPU *detector ) : detectorORB_CUDA( detector ),
    detectorSURF_CUDA( 0 ),
    detectorSURF_OCL( 0 )
{}

OpenCvGPUFeatureDetectorWrapper::OpenCvGPUFeatureDetectorWrapper( cv::ocl::SURF_OCL *detector ) : detectorSURF_OCL( detector ),
    detectorSURF_CUDA( 0 ),
    detectorORB_CUDA( 0 )
{}

OpenCvGPUFeatureDetectorWrapper::~OpenCvGPUFeatureDetectorWrapper()
{
    delete detectorSURF_CUDA;
    delete detectorORB_CUDA;
    delete detectorSURF_OCL;

    if ( detectorSURF_CUDA || detectorORB_CUDA )
        resetDevice();
}

double OpenCvGPUFeatureDetectorWrapper::getProperty( const std::string &name ) const
{
    CORE_UNUSED( name );
    return 0.0;
}

void OpenCvGPUFeatureDetectorWrapper::setProperty( const std::string &name, const double &value )
{
    CORE_UNUSED( name );
    CORE_UNUSED( value );
}

void OpenCvGPUFeatureDetectorWrapper::detectImpl( RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, int )
{
    if ( image.getType() != BufferType::U8 || !image.isValid() )
    {
        std::cerr << __LINE__ << "Invalid image type" << std::endl;
    }

	std::vector<cv::KeyPoint> kps;
    if ( detectorSURF_CUDA || detectorORB_CUDA )
    {
        cv::gpu::GpuMat img( convert( image ) );
        cv::gpu::GpuMat mask;

        if ( detectorSURF_CUDA )
            ( *detectorSURF_CUDA )( img, mask, kps );
        else if ( detectorORB_CUDA )
            ( *detectorORB_CUDA )( img, mask, kps );
    }
    else if ( detectorSURF_OCL )
    {
        cv::ocl::oclMat img( convert( image ) );
        cv::ocl::oclMat mask;
        (*detectorSURF_OCL)( img, mask, kps );
    }

	keyPoints.clear();
    FOREACH(const cv::KeyPoint &kp, kps)
    {
		keyPoints.push_back(convert(kp));
    }
}

bool FindGPUDevice( bool& cudaApi )
{
    const int numCudaDevices = getCudaEnabledDeviceCount();
    bool initProvider = false;
    for ( int idx = 0; idx < numCudaDevices; idx++ )
    {
        setDevice( idx );
        cv::gpu::DeviceInfo devInfo( idx );
        if ( devInfo.isCompatible() )
        {
            initProvider = true; // found cuda device to use
            cudaApi = true;
            break;
        }

        resetDevice();
    }

    //initProvider = false;
    if ( !initProvider ) // if no cuda device found, try use first found opencl gpus
    {
        DevicesInfo oclDevices;
        getOpenCLDevices( oclDevices, cv::ocl::CVCL_DEVICE_TYPE_GPU );
        if ( oclDevices.size() )
        {
            cv::ocl::setDevice( oclDevices[ 0 ] );
            initProvider = true;
            cudaApi = false;
        }
    }

    return initProvider;
}

void init_opencv_gpu_detectors_provider()
{
    bool cudaApi = false;
    if ( FindGPUDevice( cudaApi ) )
        FeatureDetectorProvider::getInstance().add( new OpenCvGPUFeatureDetectorProvider( cudaApi ) );
}

OpenCvGPUFeatureDetectorProvider::OpenCvGPUFeatureDetectorProvider( bool _cudaApi ) : cudaApi( _cudaApi ) {}

#define SWITCH_TYPE(str, expr) \
	if(type == #str) \
	{ \
		expr; \
	}

FeatureDetector* OpenCvGPUFeatureDetectorProvider::getFeatureDetector( const DetectorType &type, const std::string &params )
{
	SurfParams surfParams(params);
	OrbParams orbParams(params);
    if ( cudaApi )
    {
        SWITCH_TYPE( SURF_GPU,
            return new OpenCvGPUFeatureDetectorWrapper( new cv::gpu::SURF_GPU( surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, 0.01f, surfParams.upright ) ); )
        SWITCH_TYPE( ORB_GPU,
            return new OpenCvGPUFeatureDetectorWrapper( new cv::gpu::ORB_GPU( orbParams.maxFeatures, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize ) ); )
    }
    else
        SWITCH_TYPE( SURF_GPU,
            return new OpenCvGPUFeatureDetectorWrapper( new cv::ocl::SURF_OCL( surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, 0.01f, surfParams.upright ) ); )
    
    return 0;
}

bool OpenCvGPUFeatureDetectorProvider::provides( const DetectorType &type )
{
    if ( cudaApi )
    {
        SWITCH_TYPE( SURF_GPU, return true; );
        SWITCH_TYPE( ORB_GPU, return true; );
    }
    else
        SWITCH_TYPE( SURF_GPU, return true; );
    
	return false;
}

#undef SWITCH_TYPE
