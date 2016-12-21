#include "featureDetectorProvider.h"

#include "global.h"

using namespace corecvs;
using namespace std;

FeatureDetector* FeatureDetectorProvider::getDetector(const DetectorType &type, const std::string &params)
{
    for (std::vector<FeatureDetectorProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p)
    {
        if ((*p)->provides(type))
        {
            return (*p)->getFeatureDetector(type, params);
        }
    }
    CORE_ASSERT_FAIL_P(("FeatureDetectorProvider::getDetector(%s): no providers", type.c_str()));
    return 0;
}

std::vector<string> FeatureDetectorProvider::getCaps()
{
    std::vector<string> result;
    for (std::vector<FeatureDetectorProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p)
    {
        result.push_back((*p)->name());
    }
    return result;
}

void FeatureDetectorProvider::print()
{
    cout << "FeatureDetectorProvider has " << providers.size() << " providers" << endl;
    for (std::vector<FeatureDetectorProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p)
    {
        cout << "  " << (*p)->name() << endl;
    }
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
