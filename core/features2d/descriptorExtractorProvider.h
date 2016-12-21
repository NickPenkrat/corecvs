#pragma once

#include "imageKeyPoints.h"
#include "algoBase.h"


class DescriptorExtractor : public virtual AlgoBase
{
public:
    void compute(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, corecvs::RuntimeTypeBuffer &descriptors);
    virtual ~DescriptorExtractor() {}

protected:
    virtual void computeImpl(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, corecvs::RuntimeTypeBuffer &descriptors) = 0;
};

class DescriptorExtractorProviderImpl
{
public:
    virtual DescriptorExtractor* getDescriptorExtractor(const DescriptorType &type, const std::string &params = "") = 0;
    virtual bool provides(const DescriptorType &type) = 0;
    virtual std::string name() {return "unknown"; }


    virtual ~DescriptorExtractorProviderImpl() {}
};

class DescriptorExtractorProvider
{
public:
    void add(DescriptorExtractorProviderImpl *provider);
    DescriptorExtractor* getDescriptorExtractor(const DescriptorType &type, const std::string &params = "");
    std::vector<std::string> getCaps();
    void print();

    static DescriptorExtractorProvider& getInstance();
    ~DescriptorExtractorProvider();

private:
    DescriptorExtractorProvider();
    DescriptorExtractorProvider(const DescriptorExtractorProvider&);
    DescriptorExtractorProvider& operator=(const DescriptorExtractorProvider&);

    std::vector<DescriptorExtractorProviderImpl*> providers;
};
