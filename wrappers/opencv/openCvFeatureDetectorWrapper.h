#pragma once

#include "featureDetectorProvider.h"

namespace cv {
#ifdef WITH_OPENCV_3x
	class Feature2D;
	typedef Feature2D FeatureDetector;
#else
	class FeatureDetector;
#endif  
}

#ifdef WITH_OPENCV_3x
    struct SmartPtrHolder;
#endif  

class OpenCvFeatureDetectorWrapper : public FeatureDetector
{
public:
#ifdef WITH_OPENCV_3x
    OpenCvFeatureDetectorWrapper(SmartPtrHolder *holder);
#else
    OpenCvFeatureDetectorWrapper(cv::FeatureDetector *detector);  
#endif  
    
   ~OpenCvFeatureDetectorWrapper();

    double getProperty(const std::string &name) const;
    void   setProperty(const std::string &name, const double &value);

protected:
    void detectImpl(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, int nMax);

private:
    OpenCvFeatureDetectorWrapper(const OpenCvFeatureDetectorWrapper&);
    OpenCvFeatureDetectorWrapper& operator=(const OpenCvFeatureDetectorWrapper&);

    cv::FeatureDetector* detector;
#ifdef WITH_OPENCV_3x
	SmartPtrHolder* holder;
#endif  
};

extern "C"
{
    void init_opencv_detectors_provider();
}

class OpenCvFeatureDetectorProvider : public FeatureDetectorProviderImpl
{
public:
    FeatureDetector* getFeatureDetector(const DetectorType &type, const std::string &params = "");
    virtual bool provides(const DetectorType &type) override;
    virtual std::string name()  override {return "OpenCv"; }

    ~OpenCvFeatureDetectorProvider() {}
};
