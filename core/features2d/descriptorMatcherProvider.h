#ifndef DESCRIPTORMATCHERPROVIDER_H
#define DESCRIPTORMATCHERPROVIDER_H

#include "descriptorExtractorProvider.h"
#include "imageMatches.h"

#if 0
class DescriptorMatcherProvider {
	public:
		static cv::DescriptorMatcher* getDescriptorMatcher(const DescriptorType &type);
	protected:
		DescriptorMatcherProvider();
};
#endif

class DescriptorMatcher {
public:
	void knnMatch(RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch>> &matches, size_t K);
	virtual ~DescriptorMatcher() {}
protected:
	virtual void knnMatchImpl(RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch>> &matches, size_t K) = 0;
};


class DescriptorMatcherProviderImpl {
	public:
		virtual DescriptorMatcher* getDescriptorMatcher(const DetectorType &type, const DetectorsParams &params = DetectorsParams()) = 0;
		virtual bool provides(const DetectorType &type) = 0;
		virtual ~DescriptorMatcherProviderImpl() {}
	protected:
};

class DescriptorMatcherProvider {
public:
	void add(DescriptorMatcherProviderImpl *provider);
	DescriptorMatcher* getMatcher(const DescriptorType &type, const DetectorsParams &params = DetectorsParams());
	static DescriptorMatcherProvider& getInstance();
	~DescriptorMatcherProvider();
private:
	DescriptorMatcherProvider();
	DescriptorMatcherProvider(const DescriptorMatcherProvider&);
	DescriptorMatcherProvider& operator=(const DescriptorMatcherProvider&);
	std::vector<DescriptorMatcherProviderImpl*> providers;
};


#endif

