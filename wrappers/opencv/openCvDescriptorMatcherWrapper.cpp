#include "openCvDescriptorMatcherWrapper.h"

#include "openCvKeyPointsWrapper.h"

OpenCvDescriptorMatcherWrapper::OpenCvDescriptorMatcherWrapper(cv::DescriptorMatcher *matcher) : matcher(matcher)
{

}

OpenCvDescriptorMatcherWrapper::~OpenCvDescriptorMatcherWrapper()
{
    delete matcher;
}

void OpenCvDescriptorMatcherWrapper::knnMatchImpl(RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch>> &matches, size_t K)
{
    cv::Mat qd = convert(query), td = convert(train);
    std::vector<std::vector<cv::DMatch>> matches_cv;
    matcher->knnMatch(qd, td, matches_cv, K);

    matches.clear();
    matches.resize(matches_cv.size());

    for(size_t idx = 0; idx < matches.size(); ++idx)
{
        for(std::vector<cv::DMatch>::iterator m = matches_cv[idx].begin(); m != matches_cv[idx].end(); ++m)
{
            matches[idx].push_back(convert(*m));
        }
    }
}

void init_opencv_matchers_provider()
{
    DescriptorMatcherProvider::getInstance().add(new OpenCvDescriptorMatcherProvider());
}

#define SWITCH_TYPE(str, expr) \
    if(type == #str) \
    { \
        expr; \
    }

#define SWITCH_MATCHER_TYPE(str, expr) \
    if(matcher == #str) \
    { \
        expr; \
    }

DescriptorMatcher* OpenCvDescriptorMatcherProvider::getDescriptorMatcher(const DescriptorType &type, const MatcherType &matcher)
{
    SWITCH_MATCHER_TYPE(ANN,
        SWITCH_TYPE(SIFT, return new OpenCvDescriptorMatcherWrapper(new cv::FlannBasedMatcher););
        SWITCH_TYPE(SURF, return new OpenCvDescriptorMatcherWrapper(new cv::FlannBasedMatcher););
        SWITCH_TYPE(ORB, return new OpenCvDescriptorMatcherWrapper(new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20, 10, 2))););
        SWITCH_TYPE(BRISK, return new OpenCvDescriptorMatcherWrapper(new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20, 10, 2)));););
    SWITCH_MATCHER_TYPE(BF,
        SWITCH_TYPE(SIFT, return new OpenCvDescriptorMatcherWrapper(new cv::BFMatcher(cv::NORM_L2)););
        SWITCH_TYPE(SURF, return new OpenCvDescriptorMatcherWrapper(new cv::BFMatcher(cv::NORM_L2)););
        SWITCH_TYPE(ORB, return new OpenCvDescriptorMatcherWrapper(new cv::BFMatcher(cv::NORM_HAMMING)););
        SWITCH_TYPE(BRISK, return new OpenCvDescriptorMatcherWrapper(new cv::BFMatcher(cv::NORM_HAMMING));););
    assert(false);
    return 0;
}

bool OpenCvDescriptorMatcherProvider::provides(const DescriptorType &type, const MatcherType &matcher)
{
    SWITCH_MATCHER_TYPE(ANN,
        SWITCH_TYPE(SIFT, return true;);
        SWITCH_TYPE(SURF, return true;);
        SWITCH_TYPE(BRISK, return true;);
        SWITCH_TYPE(ORB, return true;););
    SWITCH_MATCHER_TYPE(BF,
        SWITCH_TYPE(SIFT, return true;);
        SWITCH_TYPE(SURF, return true;);
        SWITCH_TYPE(BRISK, return true;);
        SWITCH_TYPE(ORB, return true;););
    return false;
}

#undef SWITCH_TYPE


