#ifndef OPENCVDETECTORWRAPPER_H
#define OPENCVDETECTORWRAPPER_H

#include "featureDetectorProvider.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

class OpenCvFeatureDetectorWrapper : public FeatureDetector
{
public:
	OpenCvFeatureDetectorWrapper(cv::FeatureDetector *detector);
	double getProperty(const std::string &name) const;
	void setProperty(const std::string &name, const double &value);
	~OpenCvFeatureDetectorWrapper();
protected:
	void detectImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints);
private:
	OpenCvFeatureDetectorWrapper(const OpenCvFeatureDetectorWrapper&);
	OpenCvFeatureDetectorWrapper& operator=(const OpenCvFeatureDetectorWrapper&);
	cv::FeatureDetector* detector;
};

extern "C"
{
	void init_opencv_detectors_provider();
}

class OpenCvFeatureDetectorProvider : public FeatureDetectorProviderImpl
{
public:
	FeatureDetector* getFeatureDetector(const DetectorType &type);
	bool provides(const DetectorType &type);
	~OpenCvFeatureDetectorProvider() {}
};


#endif
