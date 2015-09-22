#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

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
size_t N = 3;

bool checkIfExists(const std::string& name)
{
    std::ifstream is;
    is.open(name, std::ios_base::in);
    return (bool)is;
}

bool detectBase(const std::string &filename)
{
    bool ok = false;
    for (size_t i = 0; i < 15; ++i)
    {
        std::cout << "Searching for " << filename << " in " << base << "  :  ";
        ok = checkIfExists(base + filename);
        if (ok)
            break;
        std::cout << "FAILED" << std::endl;
        base = ".." + (PATH_SEPARATOR + base);
    }
    if (ok) {
        std::cout << "OK!" << std::endl;
    }
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

int main(int argc, char **argv)
{
    CORE_UNUSED(argc);
    CORE_UNUSED(argv);
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

    std::string file0 = "data" PATH_SEPARATOR  "kermit_dataset" PATH_SEPARATOR "kermit000.jpg";
    if  (!detectBase(file0))
    {
        std::cout << "Unable to find data" << std::endl;
        exit(-1);
    }

    for (size_t i = 0; i < N; ++i)
    {
        std::stringstream ss;
        ss << base << "data" << PATH_SEPARATOR << "kermit_dataset" << PATH_SEPARATOR << "kermit" << std::setw(3) << std::setfill('0') << i << ".jpg";
        filenames.push_back(ss.str());
    }

    run_detector("SIFT");
    run_detector("SURF");
    run_detector("ORB");
    run_detector("SIFTGPU");

    return 0;
}
