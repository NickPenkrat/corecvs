#include "openCvDetectAndExtractWrapper.h"
#include "openCvKeyPointsWrapper.h"

#include "openCvDefaultParams.h"

#include "global.h"

#include <opencv2/features2d/features2d.hpp>       // cv::FeatureDetector
#ifdef WITH_OPENCV_3x
#   include <opencv2/xfeatures2d/nonfree.hpp>      // cv::xfeatures2d::SURF, cv::xfeatures2d::SIFT
#   include <opencv2/xfeatures2d.hpp>				// cv::xfeatures2d::STAR
#else
#   include <opencv2/nonfree/features2d.hpp>       // cv::SURF, cv::SIFT
#endif

#ifdef WITH_OPENCV_3x
struct SmartPtrDetectorHolder
{
    SmartPtrDetectorHolder() : tag(SIFT), sift() {}
    ~SmartPtrDetectorHolder() {}
    enum {
        SIFT, SURF, STAR, FAST, BRISK, ORB, AKAZE
    } tag;

    union {
        cv::Ptr< cv::xfeatures2d::SIFT >            sift;
        cv::Ptr< cv::xfeatures2d::SURF >            surf;
        cv::Ptr< cv::xfeatures2d::StarDetector >    star;
        cv::Ptr< cv::FastFeatureDetector>           fast;
        cv::Ptr< cv::BRISK >                        brisk;
        cv::Ptr< cv::ORB >                          orb;
        cv::Ptr< cv::AKAZE >                        akaze;
    };

    cv::DetectAndExtract *get() {
        switch (tag) {
        case SIFT:
            return sift.get();
        case SURF:
            return surf.get();
        case STAR:
            return star.get();
        case FAST:
            return fast.get();
        case BRISK:
            return brisk.get();
        case ORB:
            return orb.get();
        case AKAZE:
            return akaze.get();
        default:
            return nullptr;
        }
    }

    void set(cv::Ptr<cv::xfeatures2d::SIFT> value) {
        tag = SIFT;
        sift = value;
    }
    void set(cv::Ptr<cv::xfeatures2d::SURF> value) {
        tag = SURF;
        surf = value;
    }
    void set(cv::Ptr<cv::xfeatures2d::StarDetector> value) {
        tag = STAR;
        star = value;
    }
    void set(cv::Ptr<cv::FastFeatureDetector> value) {
        tag = FAST;
        fast = value;
    }
    void set(cv::Ptr<cv::BRISK> value) {
        tag = BRISK;
        brisk = value;
    }
    void set(cv::Ptr<cv::ORB> value) {
        tag = ORB;
        orb = value;
    }
    void set(cv::Ptr<cv::AKAZE> value) {
        tag = AKAZE;
        akaze = value;
    }
};

OpenCvDetectAndExtractWrapper::OpenCvDetectAndExtractWrapper(SmartPtrDetectorHolder *holder) : holder(holder)
{
    detector = holder->get();
}

OpenCvDetectAndExtractWrapper::~OpenCvDetectAndExtractWrapper()
{
    delete holder;
}

#else // !WITH_OPENCV_3x
OpenCvDetectAndExtractWrapper::OpenCvDetectAndExtractWrapper(cv::FeatureDetector *detector) : detector(detector)
{}

OpenCvDetectAndExtractWrapper::~OpenCvDetectAndExtractWrapper()
{
    delete detector;
}

#endif // !WITH_OPENCV_3x

double OpenCvDetectAndExtractWrapper::getProperty(const std::string &name) const
{
#ifdef WITH_OPENCV_3x
    CORE_UNUSED(name);
    return 0.0;
#else
    return detector->get<double>(name);
#endif
}

void OpenCvDetectAndExtractWrapper::setProperty(const std::string &name, const double &value)
{
#ifdef WITH_OPENCV_3x
    CORE_UNUSED(name);
    CORE_UNUSED(value);
#else
    detector->set(name, value);
#endif
}

void OpenCvDetectAndExtractWrapper::detectAndExtractImpl( RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors, int nMaxKeypoints )
{
    std::vector<cv::KeyPoint> kps;
    cv::Mat cv_descriptors;
    cv::Mat img = convert(image);

    detector->detectAndCompute(img, cv::Mat(), kps, cv_descriptors);

    keyPoints.clear();
    std::vector < std::pair< int, float > > sortingData;
    sortingData.resize(kps.size());

    for (int i = 0; i < kps.size(); i++)
        sortingData[i] = std::pair< int, float >(i, kps[i].response);

    std::sort(sortingData.begin(), sortingData.end(), [](const std::pair< int, float > &l, const std::pair< int, float > &r) { return l.second > r.second; });

    const int newCount = std::min((int)sortingData.size(), nMaxKeypoints);
    cv::Mat sortedDescriptors(newCount, cv_descriptors.cols, cv_descriptors.type());
    //const size_t szElement = sortedDescriptors.elemSize1();
    const size_t szElements = sortedDescriptors.elemSize1() * cv_descriptors.cols;

    for (int i = 0; i < newCount; i++)
    {
        int idx = sortingData[i].first;
        const cv::KeyPoint &kp = kps[idx];
        keyPoints.push_back(convert(kp));
        //for (int j = 0; j < cv_descriptors.cols; j++)
        //	memcpy(sortedDescriptors.ptr(i, j), cv_descriptors.ptr(idx, j), szElement);
        memcpy(sortedDescriptors.ptr(i, 0), cv_descriptors.ptr(idx, 0), szElements);
    }

    descriptors = convert(sortedDescriptors);
}

void init_opencv_detect_and_extract_provider()
{
    DetectAndExtractProvider::getInstance().add(new OpenCvDetectAndExtractProvider());
}

OpenCvDetectAndExtractProvider::OpenCvDetectAndExtractProvider() {}

#define SWITCH_TYPE(str, expr) \
    if(type == #str) \
    { \
        expr; \
    }

DetectAndExtract *OpenCvDetectAndExtractProvider::getDetector(const DetectorType &detectorType, const DescriptorType &descriptorType, const std::string &params)
{
    if(detectorType != descriptorType)
        return nullptr;
    auto type = detectorType;

    SiftParams siftParams(params);
    SurfParams surfParams(params);
    StarParams starParams(params);
    FastParams fastParams(params);
    BriskParams briskParams(params);
    OrbParams orbParams(params);
    AkazeParams akazeParams(params);
#ifdef WITH_OPENCV_3x

    SmartPtrDetectorHolder* holder = new SmartPtrDetectorHolder;
    if (type == "SIFT")
    {
        cv::Ptr< cv::xfeatures2d::SIFT > ptr = cv::xfeatures2d::SIFT::create(0, siftParams.nOctaveLayers, siftParams.contrastThreshold, siftParams.edgeThreshold, siftParams.sigma);
        holder->set(ptr);
        return new OpenCvDetectAndExtractWrapper(holder);
    }

    if (type == "SURF")
    {
        cv::Ptr< cv::xfeatures2d::SURF > ptr = cv::xfeatures2d::SURF::create(surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, surfParams.upright);
        holder->set(ptr);
        return new OpenCvDetectAndExtractWrapper(holder);
    }

    if (type == "STAR")
    {
        cv::Ptr< cv::xfeatures2d::StarDetector > ptr = cv::xfeatures2d::StarDetector::create(starParams.maxSize, starParams.responseThreshold, starParams.lineThresholdProjected, starParams.lineThresholdBinarized, starParams.supressNonmaxSize);
        holder->set(ptr);
        return new OpenCvDetectAndExtractWrapper(holder);
    }

    if (type == "FAST")
    {
        cv::Ptr< cv::FastFeatureDetector > ptr = cv::FastFeatureDetector::create(fastParams.threshold, fastParams.nonmaxSuppression);
        holder->set(ptr);
        return new OpenCvDetectAndExtractWrapper(holder);
    }

    if (type == "BRISK")
    {
        cv::Ptr< cv::BRISK > ptr = cv::BRISK::create(briskParams.thresh, briskParams.octaves, briskParams.patternScale);
        holder->set(ptr);
        return new OpenCvDetectAndExtractWrapper(holder);
    }

    if (type == "ORB")
    {
        cv::Ptr< cv::ORB > ptr = cv::ORB::create(orbParams.maxFeatures, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize);
        holder->set(ptr);
        return new OpenCvDetectAndExtractWrapper(holder);
    }

    if (type == "AKAZE")
    {
        cv::Ptr< cv::AKAZE > ptr = cv::AKAZE::create(akazeParams.descriptorType, akazeParams.descriptorSize, akazeParams.descriptorChannels, akazeParams.threshold, akazeParams.octaves, akazeParams.octaveLayers, akazeParams.diffusivity);
        holder->set(ptr);
        return new OpenCvDetectAndExtractWrapper(holder);
    }

#else
    SWITCH_TYPE(SIFT,
            return new OpenCvDetectAndExtractWrapper(new cv::SIFT(0, siftParams.nOctaveLayers, siftParams.contrastThreshold, siftParams.edgeThreshold, siftParams.sigma));)
    SWITCH_TYPE(SURF,
            return new OpenCvDetectAndExtractWrapper(new cv::SURF(surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, surfParams.upright));)
    SWITCH_TYPE(STAR,
            return new OpenCvDetectAndExtractWrapper(new cv::StarFeatureDetector(starParams.maxSize, starParams.responseThreshold, starParams.lineThresholdProjected, starParams.lineThresholdBinarized,	starParams.supressNonmaxSize));)
    SWITCH_TYPE(FAST,
            return new OpenCvDetectAndExtractWrapper(new cv::FastFeatureDetector(fastParams.threshold, fastParams.nonmaxSuppression));)
    SWITCH_TYPE(BRISK,
            return new OpenCvDetectAndExtractWrapper(new cv::BRISK(briskParams.thresh, briskParams.octaves, briskParams.patternScale));)
    SWITCH_TYPE(ORB,
            return new OpenCvDetectAndExtractWrapper(new cv::ORB(orbParams.maxFeatures, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize));)
#endif
    return 0;
}

bool OpenCvDetectAndExtractProvider::provides( const DetectorType &detectorType, const DescriptorType &descriptorType )
{
    if(descriptorType == detectorType) {
        const DetectorType &type = detectorType;
        SWITCH_TYPE(SIFT, return true;)
        SWITCH_TYPE(SURF, return true;)
        SWITCH_TYPE(STAR, return true;)
        SWITCH_TYPE(FAST, return true;)
        SWITCH_TYPE(BRISK, return true;)
        SWITCH_TYPE(ORB, return true;)
        SWITCH_TYPE(AKAZE, return true;)
    }
    return false;
}

#undef SWITCH_TYPE
