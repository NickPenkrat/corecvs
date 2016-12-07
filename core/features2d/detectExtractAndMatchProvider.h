#pragma once

#include "imageKeyPoints.h"
#include "featureMatchingPipeline.h"

class DetectExtractAndMatch
{
public:
    void detectExtractAndMatch( FeatureMatchingPipeline& pipeline, int nMaxKeypoints, int numResponcesPerPoint );
	virtual ~DetectExtractAndMatch() {}

protected:
    virtual void detectExtractAndMatchImpl( FeatureMatchingPipeline& pipeline, int nMaxKeypoints, int numResponcesPerPoint ) = 0;
};

class DetectExtractAndMatchProviderImpl
{
public:
    virtual DetectExtractAndMatch* getDetector( const DetectorType &detectorType, const DescriptorType &descriptorType, const MatcherType &matcherType, const std::string &params = "" ) = 0;
	virtual bool provides(const DetectorType &detectorType, const DescriptorType &descriptorType, const MatcherType &matcherType) = 0;

    virtual ~DetectExtractAndMatchProviderImpl() {}
};

class DetectExtractAndMatchProvider
{
public:
	static DetectExtractAndMatchProvider& getInstance();
	~DetectExtractAndMatchProvider();

    void add( DetectExtractAndMatchProviderImpl *provider );
	DetectExtractAndMatch* getDetector(const DetectorType &detectorType, const DescriptorType &descriptorType, const MatcherType &matcherType, const std::string &params = "");

private:
	DetectExtractAndMatchProvider();
	DetectExtractAndMatchProvider(const DetectExtractAndMatchProvider&);
	DetectExtractAndMatchProvider& operator=(const DetectExtractAndMatchProvider&);

    std::vector<DetectExtractAndMatchProviderImpl*> providers;
};
