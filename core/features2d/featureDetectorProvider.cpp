#include "featureDetectorProvider.h"

#include "global.h"

FeatureDetector* FeatureDetectorProvider::getDetector(const DetectorType &type, const std::string &params)
{
    for (std::vector<FeatureDetectorProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p)
    {
        if ((*p)->provides(type))
        {
            return (*p)->getFeatureDetector(type, params);
        }
    }

	//try 
	{
		CORE_ASSERT_FAIL_P(("FeatureDetectorProvider::getDetector(%s): no providers", type.c_str()));
	}
	//catch (AssertException)
	{

	}

    return 0;
}

void FeatureDetector::detect(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, int nKeypoints)
{
    detectImpl(image, keyPoints, nKeypoints);
}

FeatureDetectorProvider::~FeatureDetectorProvider()
{
    for (std::vector<FeatureDetectorProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p)
    {
        delete *p;
    }
    providers.clear();
}

void FeatureDetectorProvider::add(FeatureDetectorProviderImpl *provider)
{
    providers.push_back(provider);
}

FeatureDetectorProvider& FeatureDetectorProvider::getInstance()
{
    static FeatureDetectorProvider provider;
    return provider;
}

FeatureDetectorProvider::FeatureDetectorProvider()
{
}
