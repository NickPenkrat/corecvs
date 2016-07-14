#ifndef SIFTGPUMATCHERWRAPPER_H
#define SIFTGPUMATCHERWRAPPER_H

#include "descriptorMatcherProvider.h"

#include "SiftGPU/src/SiftGPU/SiftGPU.h"

class SiftGpuMatcher : public DescriptorMatcher
{
public:
    SiftGpuMatcher();
    SiftGpuMatcher(const SiftGpuMatcher &c);
    virtual ~SiftGpuMatcher();

    // TODO: add meaningful implementation for converting params to siftgpu's argc/argv
    double getProperty(const std::string &name) const
    {
        CORE_UNUSED(name);
        return 0.0;
    }
    void setProperty(const std::string &name, const double &value)
    {
        CORE_UNUSED(name);
        CORE_UNUSED(value);
    }
    bool isParallelable() { return false; }
protected:
    void knnMatchImpl( RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch> >& matches, size_t K);
private:
    SiftMatchGPU* initSiftMatchGpu(int count = 8192);
    SiftMatchGPU* siftMatchGpu;
};

extern "C"
{
    void init_siftgpu_matcher_provider();
}


class SiftGpuDescriptorMatcherProvider : public DescriptorMatcherProviderImpl
{
public:
    DescriptorMatcher* getDescriptorMatcher(const DescriptorType &type, const MatcherType &matcher, const std::string& params = "");
    bool provides(const DescriptorType &type, const MatcherType &matcher);
    ~SiftGpuDescriptorMatcherProvider() {}
protected:
};

#endif
