#ifndef OPENCVDETECTORWRAPPER_H
#define OPENCVDETECTORWRAPPER_H

#include "featureDetectorProvider.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

class OpenCvFeatureDetectorWrapper : public FeatureDetector {
public:
	OpenCvFeatureDetectorWrapper(cv::FeatureDetector *detector);
	~OpenCvFeatureDetectorWrapper();
protected:
	void detectImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints);
private:
	cv::FeatureDetector* detector;
	OpenCvFeatureDetectorWrapper(const OpenCvFeatureDetectorWrapper &wrapper);
};

void __attribute__ ((constructor)) __attribute__ ((used)) init_opencv_detectors_provider();

class OpenCvFeatureDetectorProvider : public FeatureDetectorProviderImpl {
	public:
		FeatureDetector* getFeatureDetector(const DetectorType &type, const DetectorsParams &params = DetectorsParams());
		bool provides(const DetectorType &type);
		~OpenCvFeatureDetectorProvider() {}
	protected:
};


#endif
