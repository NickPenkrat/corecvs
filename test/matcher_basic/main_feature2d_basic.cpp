#include <cassert>
#include <cstring>
#include <iostream>
#include <string>
#include <algorithm>

#include "featureMatchingPipeline.h"

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

void run_detector(const std::string &detector) {
	FeatureMatchingPipeline pipeline(filenames);
	pipeline.add(new KeyPointDetectionStage(DetectorType(detector), DetectorsParams()), true);
	pipeline.run();

	assert(pipeline.images[0].keyPoints.keyPoints.size());
	assert(pipeline.images[1].keyPoints.keyPoints.size());
	assert(pipeline.images[2].keyPoints.keyPoints.size());

	std::cout << detector << " detector is OK (some points were detected)" << std::endl;
}

int main(int argc, char ** argv) {
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

	filenames.push_back("./data/kermit_dataset/kermit000.jpg");
	filenames.push_back("./data/kermit_dataset/kermit001.jpg");
	filenames.push_back("./data/kermit_dataset/kermit002.jpg");

	run_detector("SIFTGPU");
	run_detector("SIFT");
	run_detector("SURF");
	run_detector("ORB");

	return 0;
}
