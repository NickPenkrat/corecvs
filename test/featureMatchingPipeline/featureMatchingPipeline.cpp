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
#       include "openCvGPUDetectAndExtractWrapper.h"
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
            std::cout << "Error: unable to find file " << fileNames[i] << std::endl;
            return false;
        }
    }

    return true;
}

void performPipelineTest( DetectorType detectorType, MatcherType matcherType, std::vector<std::string> filenames )
{
    static const uint numRuns = 2;
	std::cout << "------------------------------" << std::endl;
    std::cout << std::endl << "Running " << detectorType << " detector/descriptor and " << matcherType << " matcher " << numRuns << " times on" << std::endl;
	for (uint i = 0; i < filenames.size(); i++)
		std::cout << "\t" << filenames[i] << std::endl;

    FeatureMatchingPipeline pipeline( filenames );

#if 1
	addDetectAndExtractStage(pipeline, detectorType, detectorType);
	pipeline.add(new MatchingPlanComputationStage(), true);
	pipeline.add(new MatchAndRefineStage(detectorType, matcherType), true);
#else
    addDetectExtractAndMatchStage( pipeline, detectorType, detectorType, matcherType );
    pipeline.add( new RefineMatchesStage(), true );
#endif

#if 1
	std::chrono::time_point<std::chrono::system_clock> start, end;
    for ( uint runIdx = 0; runIdx <= numRuns; runIdx++ )
    {
        pipeline.run();
		if (!runIdx)
			start = std::chrono::system_clock::now();
    }

	end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	std::cout << "pipeline run time: " << elapsed_seconds.count() / numRuns << " seconds" << std::endl;
#else
	size_t tic, toc;
	for (uint runIdx = 0; runIdx <= numRuns; runIdx++)
	{
		pipeline.run();
		if (!runIdx)
			tic = clock();
	}

	toc = clock();
	size_t elapsed = toc - tic;
	std::cout << "pipeline run time: " << elapsed / numRuns << " ms" << std::endl;
#endif
}

int main(int /*argc*/, char ** /*argv*/)
{
	std::cout << "Running feature matching pipeline test" << std::endl;
    const char* sTopconDirEnv = "TOPCON_DIR";
    const char* sTopconDir = getenv( sTopconDirEnv );
    if ( !sTopconDir )
    {
		std::cout << "FAILED: Unable to find " << sTopconDirEnv << " terminating" << std::endl;
        return -1;
    }

    bool cudaApi = false;
    bool gpuFound = false;
    
#ifdef WITH_OPENCV
#   ifdef WITH_OPENCV_GPU
	init_opencv_gpu_detectors_provider();
	init_opencv_gpu_matchers_provider();
	init_opencv_gpu_descriptors_provider();
	gpuFound = init_opencv_gpu_detect_and_extract_provider(cudaApi);
	//gpuFound = init_opencv_gpu_detect_extract_and_match_provider(cudaApi);

	if (!gpuFound)
		std::cout << "WARNING: Unable to find compartible gpu. Only cpu versions are tested" << std::endl;

	if (gpuFound && !cudaApi)
		std::cout << "WARNING: Unable to find CUDA compartible gpu. ORB_GPU detector is not available" << std::endl;

#   endif
    init_opencv_detectors_provider();
    init_opencv_matchers_provider();
    init_opencv_reader_provider();
    init_opencv_descriptors_provider();
#endif
#ifdef WITH_SIFTGPU
    init_siftgpu_detector_provider();
    init_siftgpu_descriptor_provider();
    init_siftgpu_matcher_provider();
#endif

    std::vector<std::string> conditFileNames;
    std::vector<std::string> houseFileNames;

    std::string sConditPath = sTopconDir + std::string( "\\data\\Measure_23_canon700dFF24_MSK_GPS\\condit\\" );
    std::string sHousePath  = sTopconDir + std::string( "\\data\\Measure_23_canon700dFF24_MSK_GPS\\house\\" );

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

	std::cout << std::endl << "Finished" << std::endl;
    return 0;
}
