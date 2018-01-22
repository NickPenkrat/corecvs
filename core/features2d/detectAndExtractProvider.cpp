#include "core/features2d/detectAndExtractProvider.h"
#include "core/utils/global.h"

DetectAndExtract* DetectAndExtractProvider::getDetector( const DetectorType &detectorType, const DescriptorType &descriptorType, const std::string &params )
{
    for ( std::vector<DetectAndExtractProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p )
	{
		if ((*p)->provides(detectorType, descriptorType))
		{
			return (*p)->getDetector(detectorType, descriptorType, params);
		}
	}

	CORE_ASSERT_FAIL_P(("DetectAndExtractProvider::getDetector(%s,%s): no providers", detectorType.c_str(), descriptorType.c_str()));
	return 0;
}

void DetectAndExtract::detectAndExtract(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, corecvs::RuntimeTypeBuffer &descriptors, int nMaxKeypoints, void* pRemapCache)
{
	detectAndExtractImpl(image, keyPoints, descriptors, nMaxKeypoints, pRemapCache);
}

DetectAndExtractProvider::~DetectAndExtractProvider()
{}

DetectAndExtractProvider& DetectAndExtractProvider::getInstance()
{
    static DetectAndExtractProvider provider;
	return provider;
}

DetectAndExtractProvider::DetectAndExtractProvider()
{}
