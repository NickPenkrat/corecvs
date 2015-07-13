#ifndef FEATUREDETECTORPROVIDER_H
#define FEATUREDETECTORPROVIDER_H

#include "imageKeyPoints.h"
#include "algoBase.h"

class FeatureDetector : public virtual AlgoBase
{
public:
	void detect(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints);
	virtual ~FeatureDetector() {}
protected:
	virtual void detectImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints) = 0;
};

class FeatureDetectorProviderImpl
{
public:
	virtual FeatureDetector* getFeatureDetector(const DetectorType &type) = 0;
	virtual bool provides(const DetectorType &type) = 0;
	virtual ~FeatureDetectorProviderImpl() {}
protected:
};

class FeatureDetectorProvider
{
public:
	void add(FeatureDetectorProviderImpl *provider);
	FeatureDetector* getDetector(const DetectorType &type);
	static FeatureDetectorProvider& getInstance();
	~FeatureDetectorProvider();
private:
	FeatureDetectorProvider();
	FeatureDetectorProvider(const FeatureDetectorProvider&);
	FeatureDetectorProvider& operator=(const FeatureDetectorProvider&);
	std::vector<FeatureDetectorProviderImpl*> providers;
};


#endif
