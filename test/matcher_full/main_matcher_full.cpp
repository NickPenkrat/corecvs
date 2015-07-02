#include <cassert>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

#include "featureMatchingPipeline.h"

#ifdef WITH_OPENCV
#include "openCvFeatureDetectorWrapper.h"
#include "openCvDescriptorExtractorWrapper.h"
#include "openCvDescriptorMatcherWrapper.h"
#include "openCvFileReader.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#else
#error OpenCV is required
#endif
#ifdef WITH_SIFTGPU
#include "siftGpuWrapper.h"
#include "siftGpuMatcherWrapper.h"
#endif

std::vector<std::string> filenames;


class DrawMatchesStage : public FeatureMatchingPipelineStage {
public:
	void run(FeatureMatchingPipeline *pipeline) {
		auto images = pipeline->images;
		for(auto img: images) {
			char filename[1000];
			sprintf(filename, "%s.features.png", img.filename.c_str());

			cv::Mat src = cv::imread(img.filename);
			
			auto keyPoints = img.keyPoints.keyPoints;
			for(auto kp: keyPoints) {
				cv::circle(src, cv::Point((int)kp.x, (int)kp.y), 2, cv::Scalar(255,0,0), -2);
			}

			cv::imwrite(filename, src);
		}
	}
	void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {}
	void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {};
};

std::string base;
std::string tempBase;
int N = 11;

void detectBase() {
	std::ifstream ts;
	base = "./data/kermit_dataset/";
	ts.open("./data/kermit_dataset/kermit000.jpg", std::istream::in);
	if(!ts) {
		base = "../data/kermit_dataset/";
	}
}

void prepareCopy(const std::string &postfix) {
	char command[1000] = {0};
	sprintf(command, "mkdir kermit_%s", postfix.c_str());
//	system(std::string("mkdir kermit_") + postfix);
	system(command);
	sprintf(command, "cp %s*.jpg kermit_%s/", base.c_str(), postfix.c_str());
	system(command);
//	system(std::string("cp ") + base + std::string("*.jpg kermit_") + postfix + std::string("/"));
	tempBase = std::string("kermit_") + postfix + std::string("/");
}

void run(const std::string &detector) {
	prepareCopy(detector);
	std::vector<std::string> filenames;
	char name[1000] = {0};
	for(int i = 0; i < N; ++i) {
		sprintf(name, "%skermit%03d.jpg", tempBase.c_str(), i);
		filenames.push_back(std::string(name));
	}

	FeatureMatchingPipeline pipeline(filenames);

	pipeline.add(new KeyPointDetectionStage(DetectorType(detector), DetectorsParams()), true, std::make_pair<bool, std::string>(true, ""));
	pipeline.add(new DrawMatchesStage, true);
	pipeline.add(new DescriptorExtractionStage(DescriptorType(detector), DetectorsParams()), true, std::make_pair<bool, std::string>(true, "desc"));
	pipeline.add(new MatchingPlanComputationStage(), true, std::make_pair<bool, std::string>(true, tempBase + "plan.txt"));
	pipeline.add(new MatchingStage(DescriptorType(detector)), true, std::make_pair<bool, std::string>(true, tempBase + "raw_matches.txt"));
	pipeline.add(new RefineMatchesStage(), true, std::make_pair<bool, std::string>(true, tempBase + "refined_matches.txt"));
	pipeline.add(new VsfmWriterStage(false), true, std::make_pair<bool, std::string>(true, tempBase + "vsfm_matches.txt"));

	std::cerr << std::endl << "Running with " << detector << " detector/descriptor" << std::endl << std::endl;
	pipeline.run();
	std::cerr <<
		"Detected keypoints were saved to " << tempBase << "*.keypoints (matches are drawn on .png images)" << std::endl <<
		"Extracted descriptors were saved to " << tempBase << "*.descriptors" << std::endl <<
		"Matching plan was saved to " << tempBase << "plan.txt" << std::endl <<
		"Raw matches were saved to " << tempBase << "raw_matches.txt" << std::endl <<
		"You can now run VisualSFM in " << tempBase << " folder (import pairwise matching data from vsfm_matches.txt" << std::endl << std::endl;

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

	detectBase();
	run("SIFT");
	run("SURF");
	run("SIFTGPU");
	return 0;
}
