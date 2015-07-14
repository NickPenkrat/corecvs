#include <cassert>
#include <iostream>
#include <fstream>

#include "featureMatchingPipeline.h"

#include "global.h"

#ifdef WITH_OPENCV
#include "openCvFeatureDetectorWrapper.h"
#include "openCvDescriptorExtractorWrapper.h"
#include "openCvDescriptorMatcherWrapper.h"
#include "openCvFileReader.h"
#endif
#ifdef WITH_SIFTGPU
#include "siftGpuWrapper.h"
#include "siftGpuMatcherWrapper.h"
#endif

std::vector<std::string> filenames;
std::string base = std::string(".") + PATH_SEPARATOR;

bool checkIfExists(const std::string& name)
{
	std::ifstream is;
	is.open(name, std::ios_base::in);
	return (bool)is;
}

bool detectBase(const std::string &filename)
{
	bool ok = false;
	for(size_t i = 0; i < 15; ++i)
	{
		std::cout << "Searching for " << filename << " in " << base << "  :  ";
		if(ok = checkIfExists(base + filename)) break;
		std::cout << "FAILED" << std::endl;
		base = ".." + (PATH_SEPARATOR + base);
	}
	if(ok)
		std::cout << "OK!" << std::endl;
	return ok;
}

void run_detector(const std::string &detector)
{
	FeatureMatchingPipeline pipeline(filenames);
	pipeline.add(new KeyPointDetectionStage(DetectorType(detector)), true);
	pipeline.run();

	assert(pipeline.images[0].keyPoints.keyPoints.size());
	assert(pipeline.images[1].keyPoints.keyPoints.size());
	assert(pipeline.images[2].keyPoints.keyPoints.size());

	std::cout << detector << " detector is OK (some points were detected)" << std::endl;
}

int main(int argc, char ** argv)
{
#ifdef WITH_OPENCV
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

	if(!detectBase("./data/kermit_dataset/kermit000.jpg"))
	{
		std::cout << "Unable to find data" << std::endl;
		exit(-1);
	}

	filenames.push_back(base + "./data/kermit_dataset/kermit000.jpg");
	filenames.push_back(base + "./data/kermit_dataset/kermit001.jpg");
	filenames.push_back(base + "./data/kermit_dataset/kermit002.jpg");

	run_detector("SIFT");
	run_detector("SURF");
	run_detector("ORB");
	run_detector("SIFTGPU");

	return 0;
}
