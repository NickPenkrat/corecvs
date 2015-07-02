#ifndef FEATUREDETECTORPROVIDER_H
#define FEATUREDETECTORPROVIDER_H

#include <vector>
#include <iostream>

#include "detectorParams.h"
#include "imageKeyPoints.h"

#if 0
enum class FeatureDetectorType {
	FAST,
	STAR,
	SIFT,
	SURF,
	ORB,
	BRISK,
	SIFTGPU
};

#define DETECTOR_TYPE_FROM_STRING(str) \
	if(!strcmp(name, #str)) \
type = FeatureDetectorType::str;
#define DETECTOR_TYPE_TO_STRING(str) \
	case FeatureDetectorType::str: \
return #str;

struct DetectorType {
	FeatureDetectorType type;

	DetectorType(FeatureDetectorType type = FeatureDetectorType::SIFT) : type(type) {}
	DetectorType(const char* name) {
		init(name);
	}
	DetectorType(const std::string& name) {
		init(name.c_str());
	}
	operator FeatureDetectorType() const { return type; }
	explicit operator const char*() const {
		switch(type) {
			DETECTOR_TYPE_TO_STRING(SIFT);
			DETECTOR_TYPE_TO_STRING(SURF);
			DETECTOR_TYPE_TO_STRING(ORB);
			DETECTOR_TYPE_TO_STRING(BRISK);
			DETECTOR_TYPE_TO_STRING(FAST);
			DETECTOR_TYPE_TO_STRING(STAR);
			DETECTOR_TYPE_TO_STRING(SIFTGPU);
		}
	}
	explicit operator std::string() const {
		return std::string((const char*)(*this));
	}
private:
	void init(const char* name) {
		type = FeatureDetectorType::SURF;
		DETECTOR_TYPE_FROM_STRING(SIFT);
		DETECTOR_TYPE_FROM_STRING(SURF);
		DETECTOR_TYPE_FROM_STRING(ORB);
		DETECTOR_TYPE_FROM_STRING(BRISK);
		DETECTOR_TYPE_FROM_STRING(FAST);
		DETECTOR_TYPE_FROM_STRING(STAR);
		DETECTOR_TYPE_FROM_STRING(SIFTGPU);
	}
};
#else
#if 0
typedef std::string DetectorType;
#endif
#endif

// TODO: it seems unclear wether or not we need detectImpl instead of just 
// making detect/compute itself virtual
class FeatureDetector {
public:
	void detect(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints);
	virtual ~FeatureDetector() {}
protected:
	virtual void detectImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints) = 0;
};


class FeatureDetectorProviderImpl {
	public:
		virtual FeatureDetector* getFeatureDetector(const DetectorType &type, const DetectorsParams &params = DetectorsParams()) = 0;
		virtual bool provides(const DetectorType &type) = 0;
		virtual ~FeatureDetectorProviderImpl() {}
	protected:
};

class FeatureDetectorProvider {
public:
	void add(FeatureDetectorProviderImpl *provider);
	FeatureDetector* getDetector(const DetectorType &type, const DetectorsParams &params = DetectorsParams());
	static FeatureDetectorProvider& getInstance();
	~FeatureDetectorProvider();
private:
	FeatureDetectorProvider();
	FeatureDetectorProvider(const FeatureDetectorProvider&);
	FeatureDetectorProvider& operator=(const FeatureDetectorProvider&);
	std::vector<FeatureDetectorProviderImpl*> providers;
};


#endif
