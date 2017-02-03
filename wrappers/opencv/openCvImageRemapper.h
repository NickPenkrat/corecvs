#pragma once

#include "rgb24Buffer.h"
#include "displacementBuffer.h"

#ifdef WITH_OPENCV
#include "opencv2/core/core.hpp"
#   if defined( WITH_OPENCV_GPU ) && !defined( WITH_OPENCV_3x )
#       include <opencv2/ocl/ocl.hpp>
#       include <opencv2/gpu/gpu.hpp>
#   endif
#   if defined( WITH_OPENCV_GPU ) && defined( WITH_OPENCV_3x )
#       include <opencv2/cuda/cuda.hpp>
#   endif
#endif

namespace corecvs
{
#ifdef WITH_OPENCV
    // map0 & map1 are input params in cv::remap()
    void convert( const corecvs::DisplacementBuffer &src, cv::Mat &map0, cv::Mat &map1 );

#   ifdef WITH_OPENCV_GPU
#       ifdef WITH_OPENCV_3x
    // cuda version
    void convert( const corecvs::DisplacementBuffer &transform, cv::cuda::GpuMat &map0, cv::cuda::GpuMat &map1 ); 
    // openCL version
    void convert( const corecvs::DisplacementBuffer &transform, cv::UMat &map0, cv::UMat &map1 );
#       else
    // cuda version
    void convert( const corecvs::DisplacementBuffer &transform, cv::gpu::GpuMat &map0, cv::gpu::GpuMat &map1 );
    // openCL version
    void convert( const corecvs::DisplacementBuffer &transform, cv::ocl::oclMat &map0, cv::ocl::oclMat &map1 );
#       endif
#   endif

#endif

    void remap( corecvs::RGB24Buffer &src, corecvs::RGB24Buffer &dst, const corecvs::DisplacementBuffer &transform );
};

