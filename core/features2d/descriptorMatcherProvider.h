#ifndef DESCRIPTORMATCHERPROVIDER_H
#define DESCRIPTORMATCHERPROVIDER_H

#include "descriptorExtractorProvider.h"
#include "imageMatches.h"

class DescriptorMatcher
{
public:
	void knnMatch(RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch>> &matches, size_t K);
	virtual ~DescriptorMatcher() {}
protected:
	virtual void knnMatchImpl(RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch>> &matches, size_t K) = 0;
};


class DescriptorMatcherProviderImpl
{
public:
	virtual DescriptorMatcher* getDescriptorMatcher(const DetectorType &type) = 0;
	virtual bool provides(const DetectorType &type) = 0;
	virtual ~DescriptorMatcherProviderImpl() {}
protected:
};

class DescriptorMatcherProvider
{
public:
	void add(DescriptorMatcherProviderImpl *provider);
	DescriptorMatcher* getMatcher(const DescriptorType &type);
	static DescriptorMatcherProvider& getInstance();
	~DescriptorMatcherProvider();
private:
	DescriptorMatcherProvider();
	DescriptorMatcherProvider(const DescriptorMatcherProvider&);
	DescriptorMatcherProvider& operator=(const DescriptorMatcherProvider&);
	std::vector<DescriptorMatcherProviderImpl*> providers;
};


#endif
