#ifdef WITH_OPENCV

#pragma once

#include "detectAndExtractProvider.h"

namespace cv {
#ifdef WITH_OPENCV_3x
    class Feature2D;
    typedef Feature2D DetectAndExtract;
#else
    class DetectAndExtract;
#endif
}

#ifdef WITH_OPENCV_3x
struct SmartPtrDetectorHolder;
#endif

class OpenCvDetectAndExtractWrapper : public DetectAndExtract
{
public:
#ifdef WITH_OPENCV_3x
    OpenCvDetectAndExtractWrapper(SmartPtrDetectorHolder *holder);
#else
    OpenCvDetectAndExtractWrapper(cv::DescriptorExtractor *detector);
#endif
   ~OpenCvDetectAndExtractWrapper();

    void   setProperty(const std::string &name, const double &value);
    double getProperty(const std::string &name) const;

    bool isParallelable() { return false; }

protected:
    void detectAndExtractImpl( RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors, int nMaxKeypoints );

private:
    OpenCvDetectAndExtractWrapper(const OpenCvDetectAndExtractWrapper &wrapper);
    OpenCvDetectAndExtractWrapper& operator=( const OpenCvDetectAndExtractWrapper& );

    cv::DetectAndExtract *detector;

#ifdef WITH_OPENCV_3x
    SmartPtrDetectorHolder* holder;
#endif
};

extern "C"
{
    void init_opencv_detect_and_extract_provider();
}

class OpenCvDetectAndExtractProvider : public DetectAndExtractProviderImpl
{
public:
    OpenCvDetectAndExtractProvider();

    ~OpenCvDetectAndExtractProvider() {}

    DetectAndExtract* getDetector( const DetectorType &detectorType, const DescriptorType &descriptorType, const std::string &params = "" );
    bool provides(const DetectorType &detectorType, const DescriptorType &descriptorType);
};

#endif
