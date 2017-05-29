#pragma once

#include "imageKeyPoints.h"
#include "algoBase.h"

class FeatureDetector : public virtual AlgoBase
{
public:
    void detect(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, int nKeypoints, void* pRemapCache);
    virtual ~FeatureDetector() {}

protected:
    virtual void detectImpl(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, int nKeypoints, void* pRemapCache) = 0;
};

class FeatureDetectorProviderImpl
{
public:
    virtual FeatureDetector* getFeatureDetector(const DetectorType &type, const std::string &params = "") = 0;
    virtual bool provides(const DetectorType &type) = 0;

    virtual std::string name() {return "unknown"; }
    virtual std::vector<std::string> provideHints() {return std::vector<std::string>(); }

    virtual ~FeatureDetectorProviderImpl() {}
};

class FeatureDetectorProvider
{
public:
    static FeatureDetectorProvider& getInstance();
    ~FeatureDetectorProvider();

    void add(FeatureDetectorProviderImpl *provider);
    FeatureDetector* getDetector(const DetectorType &type, const std::string &params = "");

    virtual std::vector<std::string> getCaps();
    void print();

private:
    FeatureDetectorProvider();
    FeatureDetectorProvider(const FeatureDetectorProvider&);
    FeatureDetectorProvider& operator=(const FeatureDetectorProvider&);

    std::vector<FeatureDetectorProviderImpl*> providers;
};
