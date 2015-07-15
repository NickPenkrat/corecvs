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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#else
#error OpenCV is required
#endif
#ifdef WITH_SIFTGPU
#include "siftGpuWrapper.h"
#include "siftGpuMatcherWrapper.h"
#endif

class DrawMatchesStage : public FeatureMatchingPipelineStage
{
public:
    void run(FeatureMatchingPipeline *pipeline)
    {
        auto images = pipeline->images;
        FOREACH(const Image& img, images)

        {
            cv::Mat src = cv::imread(img.filename);

            auto keyPoints = img.keyPoints.keyPoints;
            FOREACH(const KeyPoint& kp, keyPoints)
            {
                cv::circle(src, cv::Point((int)kp.x, (int)kp.y), 2, cv::Scalar(255,0,0), -2);
            }

            cv::imwrite(img.filename + ".features.png", src);
        }
    }
    void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename)
    {}
    void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const
    {};
};

std::string base;
std::string tempBase;
int N = 11;

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

bool checkFiles()
{
    std::ifstream ts;
    std::stringstream base_name;
    base_name << "." << PATH_SEPARATOR << "data" << PATH_SEPARATOR << "kermit_dataset" << PATH_SEPARATOR;
    base = base_name.str();
    if(!detectBase("kermit000.jpg"))
    {
        std::cout << "Error: unable to find data dir" << std::endl;
        exit(0);
    }
    for(size_t i = 0; i < N; ++i)
    {
        std::stringstream ss;
        ss << base << "kermit" << std::setw(3) << std::setfill('0') << i << ".jpg";
        if(!checkIfExists(ss.str()))
        {
            std::cout << "Error: unable to find file " << ss.str() << std::endl;
            exit(0);
        }
    }
}

void prepareCopy(const std::string &postfix)
{
    std::stringstream command;
    command << "mkdir " << base << "kermit_" << postfix;
    system(command.str().c_str());
    command.str("");
#ifndef WIN32
    command << "cp ";
#else
    command << "copy /Y ";
#endif
    command << base << "*.jpg " << base << "kermit_" << postfix << PATH_SEPARATOR;
    system(command.str().c_str());
    tempBase = base + "kermit_" + postfix + PATH_SEPARATOR;
}

void run(const std::string &detector)
{
    prepareCopy(detector);
    std::vector<std::string> filenames;
    for (int i = 0; i < N; ++i)

    {
        char name[1000];
        snprintf2buf(name, "%skermit%03d.jpg", tempBase.c_str(), i);
        if(!checkIfExists(name))
        {
            std::cerr << "FAILED: Unable to find " << name << " terminating" << std::endl;
        }
        filenames.push_back(std::string(name));
    }

    FeatureMatchingPipeline pipeline(filenames);

    pipeline.add(new KeyPointDetectionStage(DetectorType(detector)), true, true);
    pipeline.add(new DrawMatchesStage, true);
    pipeline.add(new DescriptorExtractionStage(DescriptorType(detector)), true, true);
    pipeline.add(new MatchingPlanComputationStage(), true, true, tempBase + "plan.txt");
    pipeline.add(new MatchAndRefineStage(DescriptorType(detector)), true, true, tempBase + "raw_matches.txt");
    pipeline.add(new VsfmWriterStage(false), true, true, tempBase + "vsfm_matches.txt");

    std::cerr << std::endl << "Running with " << detector << " detector/descriptor" << std::endl << std::endl;
    pipeline.run();
    std::cerr
        << "Detected keypoints were saved to " << tempBase << "*.keypoints (matches are drawn on .png images)" << std::endl
        << "Extracted descriptors were saved to " << tempBase << "*.descriptors" << std::endl
        << "Matching plan was saved to " << tempBase << "plan.txt" << std::endl
        << "Raw matches were saved to " << tempBase << "raw_matches.txt" << std::endl
        << "You can now run VisualSFM in " << tempBase << " folder (import pairwise matching data from vsfm_matches.txt" << std::endl
        << std::endl;
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

    checkFiles();
    run("SIFT");
    run("SURF");
    run("SIFTGPU");
    return 0;
}
