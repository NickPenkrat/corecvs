#include "openCvGPUDescriptorExtractorWrapper.h"
#include "openCvKeyPointsWrapper.h"
#include "openCvDefaultParams.h"

#include "global.h"

#ifndef WITH_OPENCV_3x

#include <opencv2/features2d/features2d.hpp>    // cv::FeatureExtractor
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

OpenCvGPUDescriptorExtractorWrapper::OpenCvGPUDescriptorExtractorWrapper( cv::gpu::SURF_GPU *extractor ) : extractorSURF_CUDA( extractor ),
    extractorORB_CUDA( 0 ),
    extractorSURF_OCL( 0 )
{}

OpenCvGPUDescriptorExtractorWrapper::OpenCvGPUDescriptorExtractorWrapper( cv::gpu::ORB_GPU *extractor ) : extractorORB_CUDA( extractor ),
    extractorSURF_CUDA( 0 ),
    extractorSURF_OCL( 0 )
{}

OpenCvGPUDescriptorExtractorWrapper::OpenCvGPUDescriptorExtractorWrapper( cv::ocl::SURF_OCL *extractor ) : extractorSURF_OCL( extractor ),
    extractorSURF_CUDA( 0 ),
    extractorORB_CUDA( 0 )
{}

OpenCvGPUDescriptorExtractorWrapper::~OpenCvGPUDescriptorExtractorWrapper()
{
    delete extractorSURF_CUDA;
    delete extractorORB_CUDA;
    delete extractorSURF_OCL;
}

void OpenCvGPUDescriptorExtractorWrapper::computeImpl( RuntimeTypeBuffer &image
    , std::vector<KeyPoint> &keyPoints
    , RuntimeTypeBuffer &descriptors)
{
    const bool resize = false;
    float uniformScaleFactor = 1.0f;

    std::vector<cv::KeyPoint> kps;
    FOREACH(const KeyPoint& kp, keyPoints)
    {
        kps.push_back(convert(kp));
    }
    cv::Mat img = convert(image), desc;
    if ( extractorSURF_CUDA || extractorORB_CUDA )
    {    
        GpuMat cudaImage( img );
        if ( resize )
            cudaImage = downsample( cudaImage, uniformScaleFactor );

        GpuMat cudaDescriptors;
        if ( extractorSURF_CUDA )
        {
            //extractorSURF_CUDA->keypointsRatio = ( float )K / img.size().area();
            ( *extractorSURF_CUDA )( cudaImage, GpuMat(), kps, cudaDescriptors, true );
        }
            
        else if ( extractorORB_CUDA )
        {    
            ( *extractorORB_CUDA )( cudaImage, GpuMat(), kps, cudaDescriptors );
        }

        cudaDescriptors.download( desc );
    }
    else if ( extractorSURF_OCL )
    {
        oclMat oclImage( img );
        if ( resize )
            oclImage = downsample( oclImage, uniformScaleFactor );

        oclMat oclDescriptors;
        //extractorSURF_OCL->keypointsRatio = ( float )K / img.size().area();
        ( *extractorSURF_OCL )( oclImage, oclMat(), kps, oclDescriptors, true );
        oclDescriptors.download( desc );
    }

    keyPoints.clear();
    FOREACH(const cv::KeyPoint& kp, kps)
    {
        keyPoints.push_back(convert(kp));
    }

    descriptors = convert(desc);
}

void OpenCvGPUDescriptorExtractorWrapper::setProperty( const std::string &name, const double &value )
{
    CORE_UNUSED( name );
    CORE_UNUSED( value );
}

double OpenCvGPUDescriptorExtractorWrapper::getProperty( const std::string &name ) const
{
    CORE_UNUSED( name );
    return 0;
}

extern bool FindGPUDevice( bool& cudaApi );

void init_opencv_gpu_descriptors_provider()
{
    bool cudaApi = false;
    if ( FindGPUDevice( cudaApi ) )
        DescriptorExtractorProvider::getInstance().add( new OpenCvGPUDescriptorExtractorProvider( cudaApi ) );
}

#define SWITCH_TYPE(str, expr) \
    if(type == #str) \
    { \
        expr; \
    }

OpenCvGPUDescriptorExtractorProvider::OpenCvGPUDescriptorExtractorProvider( bool cudaApi ) : cudaApi( cudaApi ) {}

DescriptorExtractor* OpenCvGPUDescriptorExtractorProvider::getDescriptorExtractor( const DescriptorType &type, const std::string &params )
{
    SurfParams surfParams( params );
    OrbParams orbParams( params );
    if ( cudaApi )
    {
        SWITCH_TYPE( SURF_GPU,
            return new OpenCvGPUDescriptorExtractorWrapper( new cv::gpu::SURF_GPU( surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, 0.01f, surfParams.upright ) ); )
        
		// does not work
		//SWITCH_TYPE( ORB_GPU,
        //    return new OpenCvGPUDescriptorExtractorWrapper( new cv::gpu::ORB_GPU( orbParams.maxFeatures, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize ) ); )
    }
    else
        SWITCH_TYPE( SURF_GPU,
            return new OpenCvGPUDescriptorExtractorWrapper( new cv::ocl::SURF_OCL( surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, 0.01f, surfParams.upright ) ); )

    return 0;
}

bool OpenCvGPUDescriptorExtractorProvider::provides( const DescriptorType &type )
{
    SWITCH_TYPE(SURF_GPU, return true;);
    return false;
}

#undef SWITCH_TYPE

#else
void init_opencv_gpu_descriptors_provider() {}

#endif