#ifndef FEATUREDETECTORPROVIDER_H
#define FEATUREDETECTORPROVIDER_H

#include <vector>
#include <string>

#include "imageKeyPoints.h"

class AlgoBase { 
	virtual double getProperty(const std::string &name) const = 0;
	virtual void setProperty(const std::string &name, const double &value) = 0;
};

// TODO: it seems unclear wether or not we need detectImpl instead of just 
// making detect/compute itself virtual
class FeatureDetector : public virtual AlgoBase {
public:
	void detect(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints);
	virtual ~FeatureDetector() {}
protected:
	virtual void detectImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints) = 0;
};

class FeatureDetectorProviderImpl {
	public:
		virtual FeatureDetector* getFeatureDetector(const DetectorType &type) = 0;
		virtual bool provides(const DetectorType &type) = 0;
		virtual ~FeatureDetectorProviderImpl() {}
	protected:
};

class FeatureDetectorProvider {
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
