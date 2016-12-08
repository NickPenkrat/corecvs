#include <cstring>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#include "global.h"
#include "featureMatchingPipeline.h"

#ifdef WITH_OPENCV
#include "openCvFeatureDetectorWrapper.h"
#include "openCvDescriptorExtractorWrapper.h"
#include "openCvDescriptorMatcherWrapper.h"
#include "openCvFileReader.h"

#   ifdef WITH_OPENCV_GPU
#       include "openCvGPUDescriptorExtractorWrapper.h"
#       include "openCvGPUFeatureDetectorWrapper.h"
#       include "openCvGPUDescriptorMatcherWrapper.h"
#       include "openCvGPUDetectAndMatchWrapper.h"
#   endif

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#else
#error OpenCV is required
#endif
#ifdef WITH_SIFTGPU
#include "siftGpuWrapper.h"
#include "siftGpuMatcherWrapper.h"
#endif

bool checkIfExists(const std::string& name)
{
    std::ifstream is;
    is.open(name, std::ios_base::in);
    return (bool)is;
}

bool checkFiles( std::vector<std::string>& fileNames )
{
    for ( uint i = 0; i < fileNames.size(); i++ )
    {
        if ( !checkIfExists( fileNames[ i ] ) )
        {
            std::cerr << "Error: unable to find file " << fileNames[i] << std::endl;
            return false;
        }
    }

    return true;
}

void performPipelineTest( DetectorType detectorType, MatcherType matcherType, std::vector<std::string> filenames )
{
    static const uint numRuns = 1 + 1;
    std::cout << std::endl << "Running " << detectorType << " detector/descriptor and " << matcherType << " matcher " << numRuns << " times" << std::endl;
    FeatureMatchingPipeline pipeline( filenames );
    AddDetectExtractAndMatchStage( pipeline, detectorType, detectorType, matcherType );
    pipeline.add( new RefineMatchesStage(), true );

    for ( uint runIdx = 0; runIdx <= numRuns; runIdx++ )
    {
        pipeline.run();
    }
}

int main(int /*argc*/, char ** /*argv*/)
{
    const char* sTopconDirEnv = "TOPCON_DIR";
    const char* sTopconDir = getenv( sTopconDirEnv );
    if ( !sTopconDir )
    {
        std::cerr << "FAILED: Unable to find " << sTopconDirEnv << " terminating" << std::endl;
        return -1;
    }

    bool cudaApi = false;
    bool gpuFound = false;
    
#ifdef WITH_OPENCV
    init_opencv_detectors_provider();
    init_opencv_matchers_provider();
    init_opencv_reader_provider();
    init_opencv_descriptors_provider();

#   ifdef WITH_OPENCV_GPU
    init_opencv_gpu_detectors_provider();
    init_opencv_gpu_matchers_provider();
    init_opencv_gpu_descriptors_provider();
    gpuFound = init_opencv_gpu_detect_extract_and_match_provider( cudaApi );

    if ( !gpuFound )
        std::cerr << "WARNING: Unable to find compartible gpu. Only cpu versions are tested" << std::endl;

    if ( gpuFound && !cudaApi )
        std::cerr << "WARNING: Unable to find CUDA compartible gpu. ORB_GPU detector is not available" << std::endl;

#   endif
#endif
#ifdef WITH_SIFTGPU
    init_siftgpu_detector_provider();
    init_siftgpu_descriptor_provider();
    init_siftgpu_matcher_provider();
#endif

    std::vector<std::string> conditFileNames;
    std::vector<std::string> houseFileNames;

    std::string sConditPath = sTopconDir + std::string( "\\data\\Measure_23\\condit\\" );
    std::string sHousePath = sTopconDir + std::string( "\\data\\Measure_23\\house\\" );

    static const uint   numImages = 2;
    const char*         conditImageNames[] = { "IMG_0287.JPG", "IMG_0290.JPG" };
    const char*         houseImageNames [] = { "IMG_0236.JPG", "IMG_0239.JPG" };
    
    conditFileNames.resize( numImages );
    houseFileNames.resize( numImages );

    for ( uint i = 0; i < numImages; i++ )
    {
        conditFileNames[ i ] = sConditPath + conditImageNames[ i ];
        houseFileNames [ i ] = sHousePath  + houseImageNames [ i ];
    }

    if ( !checkFiles( conditFileNames ) || !checkFiles( houseFileNames ) )
        return 0;

    performPipelineTest( "SURF", "BF", conditFileNames );
    if ( gpuFound )
        performPipelineTest( "SURF_GPU", "BF_GPU", conditFileNames );

    performPipelineTest( "ORB", "BF", conditFileNames );

    if ( cudaApi )
        performPipelineTest( "ORB_GPU", "BF_GPU", conditFileNames );

    performPipelineTest( "SURF", "BF", houseFileNames );

    if ( gpuFound )
        performPipelineTest( "SURF_GPU", "BF_GPU", houseFileNames );

    performPipelineTest( "ORB", "BF", houseFileNames );

    if ( cudaApi )
        performPipelineTest( "ORB_GPU", "BF_GPU", houseFileNames );

    return 0;
}
