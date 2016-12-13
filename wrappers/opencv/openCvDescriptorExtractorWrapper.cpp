#include "openCvDescriptorExtractorWrapper.h"
#include "openCvKeyPointsWrapper.h"
#include "openCvDefaultParams.h"

#include "global.h"

#include <opencv2/features2d/features2d.hpp>    // cv::DescriptorExtractor

#ifdef WITH_OPENCV_3x
#   include <opencv2/xfeatures2d/nonfree.hpp>      // cv::xfeatures2d::SURF, cv::xfeatures2d::SIFT
#   include <opencv2/xfeatures2d.hpp>				// cv::xfeatures2d::STAR
#else
#   include <opencv2/nonfree/features2d.hpp>       // cv::SURF
#endif

#ifdef WITH_OPENCV_3x
struct SmartPtrExtractorHolder
{
    cv::Ptr< cv::xfeatures2d::SIFT >            sift;
    cv::Ptr< cv::xfeatures2d::SURF >            surf;
    //cv::Ptr< cv::xfeatures2d::StarDetector >	star;
    //cv::Ptr< cv::FastFeatureDetector>         fast;
    cv::Ptr< cv::BRISK >                        brisk;
    cv::Ptr< cv::ORB >                          orb;
};

OpenCvDescriptorExtractorWrapper::OpenCvDescriptorExtractorWrapper(SmartPtrExtractorHolder *holder) : holder(holder)
{
    extractor = holder->sift.get();
    if (!extractor)
        extractor = holder->surf.get();
    //if (!extractor)
    //    extractor = holder->star.get();
    //if (!extractor)
    //    extractor = holder->fast.get();
    if (!extractor)
        extractor = holder->brisk.get();
    if (!extractor)
        extractor = holder->orb.get();
}

OpenCvDescriptorExtractorWrapper::~OpenCvDescriptorExtractorWrapper()
{
    delete holder;
}

#else // !WITH_OPENCV_3x

OpenCvDescriptorExtractorWrapper::OpenCvDescriptorExtractorWrapper(cv::DescriptorExtractor *extractor)
    : extractor(extractor)
{}

OpenCvDescriptorExtractorWrapper::~OpenCvDescriptorExtractorWrapper()
{
    delete extractor;
}

#endif

void OpenCvDescriptorExtractorWrapper::computeImpl(RuntimeTypeBuffer &image
    , std::vector<KeyPoint> &keyPoints
    , RuntimeTypeBuffer &descriptors)
{
    std::vector<cv::KeyPoint> kps;
    FOREACH(const KeyPoint& kp, keyPoints)
    {
        kps.push_back(convert(kp));
    }
    cv::Mat img = convert(image), desc;

    extractor->compute(img, kps, desc);

    keyPoints.clear();
    FOREACH(const cv::KeyPoint& kp, kps)
    {
        keyPoints.push_back(convert(kp));
    }

    descriptors = convert(desc);
}

void OpenCvDescriptorExtractorWrapper::setProperty(const std::string &name, const double &value)
{
#ifdef WITH_OPENCV_3x
    CORE_UNUSED(name);
    CORE_UNUSED(value);
#else
    extractor->set(name, value);
#endif
}

double OpenCvDescriptorExtractorWrapper::getProperty(const std::string &name) const
{
#ifdef WITH_OPENCV_3x
    CORE_UNUSED(name);
    return 0.0;
#else
    return extractor->get<double>(name);
#endif
}

void init_opencv_descriptors_provider()
{
    DescriptorExtractorProvider::getInstance().add(new OpenCvDescriptorExtractorProvider());
}

#define SWITCH_TYPE(str, expr) \
    if(type == #str) \
    { \
        expr; \
    }

DescriptorExtractor* OpenCvDescriptorExtractorProvider::getDescriptorExtractor(const DescriptorType &type, const std::string &params)
{
    SiftParams siftParams(params);
    SurfParams surfParams(params);
    BriskParams briskParams(params);
    OrbParams orbParams(params);
#ifdef WITH_OPENCV_3x
	SmartPtrExtractorHolder* holder = new SmartPtrExtractorHolder;
    if (type == "SIFT")
    {
        cv::Ptr< cv::xfeatures2d::SIFT > ptr = cv::xfeatures2d::SIFT::create(0, siftParams.nOctaveLayers, siftParams.contrastThreshold, siftParams.edgeThreshold, siftParams.sigma);
        holder->sift = ptr;
        return new OpenCvDescriptorExtractorWrapper(holder);
    }

    if (type == "SURF")
    {
        cv::Ptr< cv::xfeatures2d::SURF > ptr = cv::xfeatures2d::SURF::create(surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, surfParams.upright);
        holder->surf = ptr;
        return new OpenCvDescriptorExtractorWrapper(holder);
    }

    if (type == "BRISK")
    {
        cv::Ptr< cv::BRISK > ptr = cv::BRISK::create(briskParams.thresh, briskParams.octaves, briskParams.patternScale);
        holder->brisk = ptr;
        return new OpenCvDescriptorExtractorWrapper(holder);
    }

    if (type == "ORB")
    {
        cv::Ptr< cv::ORB > ptr = cv::ORB::create(orbParams.maxFeatures, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize);
        holder->orb = ptr;
        return new OpenCvDescriptorExtractorWrapper(holder);
    }

#else
    SWITCH_TYPE(SIFT,
        return new OpenCvDescriptorExtractorWrapper(new cv::SIFT(0, siftParams.nOctaveLayers, siftParams.contrastThreshold, siftParams.edgeThreshold, siftParams.sigma));)
    SWITCH_TYPE(SURF,
        return new OpenCvDescriptorExtractorWrapper(new cv::SURF(surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, surfParams.upright));)
    SWITCH_TYPE(BRISK,
        return new OpenCvDescriptorExtractorWrapper(new cv::BRISK(briskParams.thresh, briskParams.octaves, briskParams.patternScale));)
    SWITCH_TYPE(ORB,
        return new OpenCvDescriptorExtractorWrapper(new cv::ORB(orbParams.maxFeatures, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize));)
#endif
     return 0;
}

bool OpenCvDescriptorExtractorProvider::provides(const DescriptorType &type)
{
    SWITCH_TYPE(SIFT, return true;);
    SWITCH_TYPE(SURF, return true;);
    SWITCH_TYPE(BRISK, return true;);
    SWITCH_TYPE(ORB, return true;);
    return false;
}

#undef SWITCH_TYPE
