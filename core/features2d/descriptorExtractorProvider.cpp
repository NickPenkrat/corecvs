#include "descriptorExtractorProvider.h"

#include "global.h"


DescriptorExtractor* DescriptorExtractorProvider::getDescriptorExtractor(const DescriptorType &type, const std::string &params)
{
    for (std::vector<DescriptorExtractorProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p)
    {
        if ((*p)->provides(type))
        {
            return (*p)->getDescriptorExtractor(type, params);
        }
    }
    CORE_ASSERT_FAIL_P(("DescriptorExtractorProvider::getDescriptorExtractor(%s): no providers", type.c_str()));
    return 0;
}

std::vector<std::string> DescriptorExtractorProvider::getCaps()
{
    std::vector<std::string> result;
    for (std::vector<DescriptorExtractorProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p)
    {
        result.push_back((*p)->name());
    }
    return result;
}


void DescriptorExtractorProvider::print()
{
    cout << "DescriptorExtractorProvider has " << providers.size() << " providers" << std::endl;
    for (std::vector<DescriptorExtractorProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p)
    {
        cout << "  " << (*p)->name() << std::endl;
    }
}


void DescriptorExtractorProvider::add(DescriptorExtractorProviderImpl *provider)
{
    providers.push_back(provider);
}

DescriptorExtractorProvider::~DescriptorExtractorProvider()
{
    for (std::vector<DescriptorExtractorProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p)
    {
        delete *p;
    }
    providers.clear();
}

DescriptorExtractorProvider& DescriptorExtractorProvider::getInstance()
{
    static DescriptorExtractorProvider provider;
    return provider;
}

DescriptorExtractorProvider::DescriptorExtractorProvider()
{
}

void DescriptorExtractor::compute(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, corecvs::RuntimeTypeBuffer &buffer, void* pRemapCache)
{
	computeImpl(image, keyPoints, buffer, pRemapCache);
}
