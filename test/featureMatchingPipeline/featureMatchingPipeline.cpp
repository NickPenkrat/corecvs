#include <cstring>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#include "global.h"
#include "featureMatchingPipeline.h"
#include "trackPainter.h"

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

void performPipelineTest( DetectorType detectorType, MatcherType matcherType, int downsample, std::vector<std::string> filenames )
{
    static const uint numRuns = 2;
	std::cout << "------------------------------" << std::endl;
    std::cout << std::endl << "Running " << detectorType << " detector/descriptor and " << matcherType << " matcher " << numRuns << " times on" << std::endl;
	for (uint i = 0; i < filenames.size(); i++)
		std::cout << "\t" << filenames[i] << std::endl;

    FeatureMatchingPipeline pipeline( filenames );

    addDetectAndExtractStage( pipeline, detectorType, detectorType, 4000, downsample, "", false );
	pipeline.add(new MatchingPlanComputationStage(), true);
	pipeline.add(new MatchAndRefineStage(detectorType, matcherType), true);

	std::chrono::time_point<std::chrono::system_clock> start, end;
    for ( uint runIdx = 0; runIdx <= numRuns; runIdx++ )
    {
        pipeline.run();
		if (!runIdx)
			start = std::chrono::system_clock::now();
    }

	end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	std::cout << "pipeline run time: " << elapsed_seconds.count() / numRuns << " seconds, " << pipeline.refinedMatches.matchSets[0].matches.size() << " refined matches" << std::endl;
}

static std::string getFiledir(const std::string &imgName)
{
	int pos = (int)imgName.find_last_of(PATH_SEPARATOR[0]);
	std::string res = imgName.substr(0, pos + 1);
	return res;
}

void performFastMatchingTest(DetectorType detectorType, MatcherType matcherType, int downsample, std::vector<std::string> filenames)
{
	static const uint numRuns = 2;
	std::cout << "------------------------------" << std::endl;
	std::cout << std::endl << "Running " << detectorType << " detector/descriptor and " << matcherType << " matcher " << numRuns << " times on" << std::endl;
	for (uint i = 0; i < filenames.size(); i++)
		std::cout << "\t" << filenames[i] << std::endl;

	std::vector<std::string> refFilename;
	std::vector<std::string> otherFilename;
	refFilename.push_back(filenames[0]);
	FeatureMatchingPipeline detectReferenceFeaturesPipeline(refFilename);
	addDetectAndExtractStage(detectReferenceFeaturesPipeline, detectorType, detectorType, 4000, downsample, "", false);
	otherFilename.push_back(filenames[1]);
	FeatureMatchingPipeline detectionPipeline(otherFilename);
	addDetectAndExtractStage(detectionPipeline, detectorType, detectorType, 4000, downsample, "", false);
	FeatureMatchingPipeline matchingPipeline(filenames);
	matchingPipeline.add(new MatchingPlanComputationStage(), true);
	matchingPipeline.add(new MatchAndRefineStage(detectorType, matcherType), true);
	matchingPipeline.detectorType = detectorType;
	matchingPipeline.descriptorType = detectorType;

	std::chrono::time_point<std::chrono::system_clock> start, end;
	detectReferenceFeaturesPipeline.run();

	for (uint runIdx = 0; runIdx <= numRuns; runIdx++)
	{
		detectionPipeline.run();
		matchingPipeline.images.clear();
		matchingPipeline.images.push_back(detectReferenceFeaturesPipeline.images[0]);
		matchingPipeline.images.push_back(detectionPipeline.images[0]);
		matchingPipeline.run();
		if (!runIdx)
			start = std::chrono::system_clock::now();
	}

	end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	std::cout << "pipeline run time: " << elapsed_seconds.count() / numRuns << " seconds, " << matchingPipeline.refinedMatches.matchSets[0].matches.size() << " refined matches" << std::endl;

	std::stringstream ss;
	ss << detectorType << ' ' << matcherType;
	if (downsample > 1)
		ss << " downsample " << downsample;

	for (size_t j = 0; j < matchingPipeline.refinedMatches.matchSets.size(); j++)
	{
		std::vector<std::string> imageNames;
		RefinedMatchSet& set = matchingPipeline.refinedMatches.matchSets[j];
		Image imgA = matchingPipeline.images[set.imgA];
		Image imgB = matchingPipeline.images[set.imgB];
		imageNames.push_back(imgA.filename);
		imageNames.push_back(imgB.filename);
		
		std::vector< cvs::Match > matches;
		matches.resize(set.matches.size());
		std::vector<std::vector<Vector2dd>> keypoints;
		keypoints.resize(set.matches.size());

		for (uint i = 0; i < set.matches.size(); i++)
		{	
			std::vector<Vector2dd> keypointsPerMatch;
			Vector2dd point(imgA.keyPoints.keyPoints[i].x() * downsample, imgA.keyPoints.keyPoints[i].y() * downsample);
			keypointsPerMatch.push_back( point );

			point = Vector2dd(imgB.keyPoints.keyPoints[i].x() * downsample, imgB.keyPoints.keyPoints[i].y() * downsample);
			keypointsPerMatch.push_back( point );

			keypoints[i] = keypointsPerMatch;
		}

		for (uint i = 0; i < set.matches.size(); i++)
			matches[i] = &keypoints[i];

		cvs::TrackPainter painter(imageNames, matches);
		painter.paintTracksOnImages(true, ss.str());
	}
}

int main(int argc, char ** argv)
{
    int downsample = 1;
    if ( argc >= 2 )
        downsample = atoi( argv[ 1 ] );

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

	if (false)
	{
		performPipelineTest("SURF", "BF", downsample, conditFileNames);
		if (gpuFound)
			performPipelineTest("SURF_GPU", "BF_GPU", downsample, conditFileNames);

		performPipelineTest("ORB", "BF", downsample, conditFileNames);

		//if ( cudaApi )
		//    performPipelineTest( "ORB_GPU", "BF_GPU", downsample, conditFileNames );

		performPipelineTest("SURF", "BF", downsample, houseFileNames);

		if (gpuFound)
			performPipelineTest("SURF_GPU", "BF_GPU", downsample, houseFileNames);

		performPipelineTest("ORB", "BF", downsample, houseFileNames);

		//if ( cudaApi )
		//    performPipelineTest( "ORB_GPU", "BF_GPU", downsample, houseFileNames );

	}
	else
	{		
		performFastMatchingTest("SURF", "BF", downsample, conditFileNames);
		if (gpuFound)
			performFastMatchingTest("SURF_GPU", "BF_GPU", downsample, conditFileNames);

		performFastMatchingTest("ORB", "BF", downsample, conditFileNames);

		//if ( cudaApi )
		//    performFastMatchingTest( "ORB_GPU", "BF_GPU", downsample, conditFileNames );

		performFastMatchingTest("SURF", "BF", downsample, houseFileNames);

		if (gpuFound)
			performFastMatchingTest("SURF_GPU", "BF_GPU", downsample, houseFileNames);

		performFastMatchingTest("ORB", "BF", downsample, houseFileNames);

		//if ( cudaApi )
		//    performFastMatchingTest( "ORB_GPU", "BF_GPU", downsample, houseFileNames );

	}

	std::cout << std::endl << "Finished" << std::endl;
    return 0;
}
