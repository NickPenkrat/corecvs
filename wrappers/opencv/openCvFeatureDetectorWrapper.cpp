#include "openCvFeatureDetectorWrapper.h"
#include "openCvKeyPointsWrapper.h"

#include "openCvDefaultParams.h"

#include "global.h"

#include <opencv2/features2d/features2d.hpp>    // cv::FeatureDetector
#ifdef WITH_OPENCV_3x
#   include <opencv2/xfeatures2d/nonfree.hpp>      // cv::xfeatures2d::SURF, cv::xfeatures2d::SIFT
#   include <opencv2/xfeatures2d.hpp>				// cv::xfeatures2d::STAR
#else
#   include <opencv2/nonfree/features2d.hpp>       // cv::SURF, cv::SIFT
#endif

#ifdef WITH_OPENCV_3x
struct SmartPtrDetectorHolder
{
	cv::Ptr< cv::xfeatures2d::SIFT >            sift;
	cv::Ptr< cv::xfeatures2d::SURF >            surf;
	cv::Ptr< cv::xfeatures2d::StarDetector >	star;
	cv::Ptr< cv::FastFeatureDetector>           fast;
	cv::Ptr< cv::BRISK >                        brisk;
    cv::Ptr< cv::ORB >                          orb;
};

OpenCvFeatureDetectorWrapper::OpenCvFeatureDetectorWrapper(SmartPtrDetectorHolder *holder) : holder(holder)
{
    detector = holder->sift.get();
    if (!detector)
        detector = holder->surf.get();
    if (!detector)
        detector = holder->star.get();
    if (!detector)
        detector = holder->fast.get();
    if (!detector)
        detector = holder->brisk.get();
    if (!detector)
        detector = holder->orb.get();
}

OpenCvFeatureDetectorWrapper::~OpenCvFeatureDetectorWrapper()
{
    delete holder;
}

#else // !WITH_OPENCV_3x
OpenCvFeatureDetectorWrapper::OpenCvFeatureDetectorWrapper(cv::FeatureDetector *detector) : detector(detector)
{}

OpenCvFeatureDetectorWrapper::~OpenCvFeatureDetectorWrapper()
{
    delete detector;
}

#endif // !WITH_OPENCV_3x

double OpenCvFeatureDetectorWrapper::getProperty(const std::string &name) const
{
#ifdef WITH_OPENCV_3x
	CORE_UNUSED(name);
	return 0.0;
#else
	return detector->get<double>(name);
#endif
}

void OpenCvFeatureDetectorWrapper::setProperty(const std::string &name, const double &value)
{
#ifdef WITH_OPENCV_3x
	CORE_UNUSED(name);
	CORE_UNUSED(value);
#else
	detector->set(name, value);
#endif
}

void OpenCvFeatureDetectorWrapper::detectImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, int nKeyPoints)
{
	std::vector<cv::KeyPoint> kps;
	cv::Mat img = convert(image);
  
	detector->detect(img, kps);

	keyPoints.clear();

    FOREACH(const cv::KeyPoint &kp, kps)
    {
		keyPoints.push_back(convert(kp));
    }
	std::sort(keyPoints.begin(), keyPoints.end(), [](const KeyPoint &l, const KeyPoint &r) { return l.response > r.response; });
	keyPoints.resize(std::min((int)keyPoints.size(), nKeyPoints));
}

void init_opencv_detectors_provider()
{
	FeatureDetectorProvider::getInstance().add(new OpenCvFeatureDetectorProvider());
}

#define SWITCH_TYPE(str, expr) \
	if(type == #str) \
	{ \
		expr; \
	}

FeatureDetector* OpenCvFeatureDetectorProvider::getFeatureDetector(const DetectorType &type, const std::string &params)
{
	SiftParams siftParams(params);
	SurfParams surfParams(params);
	StarParams starParams(params);
	FastParams fastParams(params);
	BriskParams briskParams(params);
	OrbParams orbParams(params);
#ifdef WITH_OPENCV_3x

	SmartPtrDetectorHolder* holder = new SmartPtrDetectorHolder;
	if (type == "SIFT")
	{
		cv::Ptr< cv::xfeatures2d::SIFT > ptr = cv::xfeatures2d::SIFT::create(0, siftParams.nOctaveLayers, siftParams.contrastThreshold, siftParams.edgeThreshold, siftParams.sigma);
        holder->sift = ptr;
        return new OpenCvFeatureDetectorWrapper(holder);
	}

    if (type == "SURF")
    {
        cv::Ptr< cv::xfeatures2d::SURF > ptr = cv::xfeatures2d::SURF::create(surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, surfParams.upright);
        holder->surf = ptr;
        return new OpenCvFeatureDetectorWrapper(holder);
    }

    if (type == "STAR")
    {
        cv::Ptr< cv::xfeatures2d::StarDetector > ptr = cv::xfeatures2d::StarDetector::create(starParams.maxSize, starParams.responseThreshold, starParams.lineThresholdProjected, starParams.lineThresholdBinarized, starParams.supressNonmaxSize);
        holder->star = ptr;
        return new OpenCvFeatureDetectorWrapper(holder);
    }

    if (type == "FAST")
    {
        cv::Ptr< cv::FastFeatureDetector > ptr = cv::FastFeatureDetector::create(fastParams.threshold, fastParams.nonmaxSuppression);
        holder->fast = ptr;
        return new OpenCvFeatureDetectorWrapper(holder);
    }

    if (type == "BRISK")
    {
        cv::Ptr< cv::BRISK > ptr = cv::BRISK::create(briskParams.thresh, briskParams.octaves, briskParams.patternScale);
        holder->brisk = ptr;
        return new OpenCvFeatureDetectorWrapper(holder);
    }

    if (type == "ORB")
    {
        cv::Ptr< cv::ORB > ptr = cv::ORB::create(orbParams.maxFeatures, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize);
        holder->orb = ptr;
        return new OpenCvFeatureDetectorWrapper(holder);
    }

#else
	SWITCH_TYPE(SIFT,
			return new OpenCvFeatureDetectorWrapper(new cv::SIFT(0, siftParams.nOctaveLayers, siftParams.contrastThreshold, siftParams.edgeThreshold, siftParams.sigma));)
	SWITCH_TYPE(SURF,
			return new OpenCvFeatureDetectorWrapper(new cv::SURF(surfParams.hessianThreshold, surfParams.octaves, surfParams.octaveLayers, surfParams.extended, surfParams.upright));)
	SWITCH_TYPE(STAR,
			return new OpenCvFeatureDetectorWrapper(new cv::StarFeatureDetector(starParams.maxSize, starParams.responseThreshold, starParams.lineThresholdProjected, starParams.lineThresholdBinarized,	starParams.supressNonmaxSize));)
	SWITCH_TYPE(FAST,
			return new OpenCvFeatureDetectorWrapper(new cv::FastFeatureDetector(fastParams.threshold, fastParams.nonmaxSuppression));)
	SWITCH_TYPE(BRISK,
			return new OpenCvFeatureDetectorWrapper(new cv::BRISK(briskParams.thresh, briskParams.octaves, briskParams.patternScale));)
	SWITCH_TYPE(ORB,
            return new OpenCvFeatureDetectorWrapper(new cv::ORB(orbParams.maxFeatures, orbParams.scaleFactor, orbParams.nLevels, orbParams.edgeThreshold, orbParams.firstLevel, orbParams.WTA_K, orbParams.scoreType, orbParams.patchSize));)
#endif
	return 0;
}

bool OpenCvFeatureDetectorProvider::provides(const DetectorType &type)
{
	SWITCH_TYPE(SIFT, return true;);
	SWITCH_TYPE(SURF, return true;);
	SWITCH_TYPE(STAR, return true;);
	SWITCH_TYPE(FAST, return true;);
	SWITCH_TYPE(BRISK, return true;);
	SWITCH_TYPE(ORB, return true;);
	return false;
}

#undef SWITCH_TYPE
