#ifndef DETECTEXTRACTANDMATCHPROVIDER_H
#define DETECTEXTRACTANDMATCHPROVIDER_H

#include "imageKeyPoints.h"
#include "featureMatchingPipeline.h"

class DetectExtractAndMatch
{
public:
    void detectExtractAndMatch(FeatureMatchingPipeline& pipeline, int nMaxKeypoints, int numResponcesPerPoint);
	virtual ~DetectExtractAndMatch() {}

protected:
    virtual void detectExtractAndMatchImpl(FeatureMatchingPipeline& pipeline, int nMaxKeypoints, int numResponcesPerPoint) = 0;
};

class DetectExtractAndMatchProviderImpl : public AlgoBase
{
public:
    virtual DetectExtractAndMatch* getDetector(const DetectorType &detectorType, const DescriptorType &descriptorType, const MatcherType &matcherType, const std::string &params = "") = 0;

    virtual bool provides(const DetectorType &detectorType, const DescriptorType &descriptorType, const MatcherType &matcherType) = 0;

    virtual ~DetectExtractAndMatchProviderImpl() {}
};

class DetectExtractAndMatchProvider : public AlgoCollectionNaming<DetectExtractAndMatchProviderImpl>
{
public:
	static DetectExtractAndMatchProvider& getInstance();

	~DetectExtractAndMatchProvider();
   
	DetectExtractAndMatch* getDetector(const DetectorType &detectorType, const DescriptorType &descriptorType, const MatcherType &matcherType, const std::string &params = "");

private:
	DetectExtractAndMatchProvider();
	DetectExtractAndMatchProvider(const DetectExtractAndMatchProvider&);
  //DetectExtractAndMatchProvider& operator=(const DetectExtractAndMatchProvider&);
};

#endif // DETECTEXTRACTANDMATCHPROVIDER_H
