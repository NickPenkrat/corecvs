#include "detectExtractAndMatchProvider.h"
#include "global.h"

DetectExtractAndMatch* DetectExtractAndMatchProvider::getDetector(const DetectorType &detectorType, const DescriptorType &descriptorType, const MatcherType &matcherType, const std::string &params)
{
    for (std::vector<DetectExtractAndMatchProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p)
	{
		if ((*p)->provides(detectorType, descriptorType, matcherType))
		{
			return (*p)->getDetector(detectorType, descriptorType, matcherType, params);
		}
	}

	//try
	{
		CORE_ASSERT_FAIL_P(("DetectExtractAndMatchProvider::getDetector(%s,%s,%s): no providers", detectorType.c_str(), descriptorType.c_str(), matcherType.c_str()));
	}
	//catch (AssertException)
	{
	
	}

	return nullptr;
}

void DetectExtractAndMatch::detectExtractAndMatch(FeatureMatchingPipeline& pipeline, int nMaxKeypoints, int numResponcesPerPoint)
{
    detectExtractAndMatchImpl(pipeline, nMaxKeypoints, numResponcesPerPoint);
}

DetectExtractAndMatchProvider::~DetectExtractAndMatchProvider()
{}

DetectExtractAndMatchProvider& DetectExtractAndMatchProvider::getInstance()
{
	static DetectExtractAndMatchProvider provider;
	return provider;
}

DetectExtractAndMatchProvider::DetectExtractAndMatchProvider()
{}
