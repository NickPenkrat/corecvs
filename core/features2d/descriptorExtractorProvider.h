#ifndef DESCRIPTOREXTRACTORPROVIDER_H
#define DESCRIPTOREXTRACTORPROVIDER_H

#include <cassert>

#include "detectorParams.h"
#include "imageKeyPoints.h"


class DescriptorExtractor {
public:
	void compute(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors);
	virtual ~DescriptorExtractor() {}
protected:
	virtual void computeImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors) = 0;
};

class DescriptorExtractorProviderImpl {
	public:
		virtual DescriptorExtractor* getDescriptorExtractor(const DescriptorType &type, const DetectorsParams &params = DetectorsParams()) = 0;
		virtual bool provides(const DescriptorType &type) = 0;
		virtual ~DescriptorExtractorProviderImpl() {}
	protected:
};

class DescriptorExtractorProvider {
public:
	void add(DescriptorExtractorProviderImpl *provider);
	DescriptorExtractor* getDescriptorExtractor(const DescriptorType &type, const DetectorsParams &params);
	static DescriptorExtractorProvider& getInstance();
	~DescriptorExtractorProvider();
private:
	DescriptorExtractorProvider();
	DescriptorExtractorProvider(const DescriptorExtractorProvider&);
	DescriptorExtractorProvider& operator=(const DescriptorExtractorProvider&);
	std::vector<DescriptorExtractorProviderImpl*> providers;
};


#endif
