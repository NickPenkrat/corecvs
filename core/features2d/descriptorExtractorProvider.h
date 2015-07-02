#ifndef DESCRIPTOREXTRACTORPROVIDER_H
#define DESCRIPTOREXTRACTORPROVIDER_H

#include <cassert>

#if 0
#include <opencv2/features2d/features2d.hpp>
#endif

#include "detectorParams.h"
#include "imageKeyPoints.h"

#if 0
enum class DescriptorExtractorType {
	SIFT,
	SURF,
	ORB,
	BRISK,
	SIFTGPU
};

#define DESCRIPTOR_TYPE_FROM_STRING(str) \
	if(!strcmp(name, #str)) \
type = DescriptorExtractorType::str;
#define DESCRIPTOR_TYPE_TO_STRING(str) \
	case DescriptorExtractorType::str: \
return #str;

struct DescriptorType {
	DescriptorExtractorType type;

	DescriptorType(DescriptorExtractorType type = DescriptorExtractorType::SIFT) : type(type) {}
	DescriptorType(const char* name) {
		init(name);
	}
	DescriptorType(const std::string &name) {
		init(name.c_str());
	}
	operator DescriptorExtractorType() const { return type; }
	explicit operator const char*() const {
		switch(type) {
			DESCRIPTOR_TYPE_TO_STRING(SIFT);
			DESCRIPTOR_TYPE_TO_STRING(SURF);
			DESCRIPTOR_TYPE_TO_STRING(ORB);
			DESCRIPTOR_TYPE_TO_STRING(BRISK);
			DESCRIPTOR_TYPE_TO_STRING(SIFTGPU);
		}
		assert(false);	
		return "";
	}
	explicit operator std::string() const {
		return std::string((const char*)(*this));
	}
private:
	void init(const char* name) {
		type = DescriptorExtractorType::SIFT;
		DESCRIPTOR_TYPE_FROM_STRING(SIFT);
		DESCRIPTOR_TYPE_FROM_STRING(SURF);
		DESCRIPTOR_TYPE_FROM_STRING(ORB);
		DESCRIPTOR_TYPE_FROM_STRING(BRISK);
		DESCRIPTOR_TYPE_FROM_STRING(SIFTGPU);
	}
};
#else
#if 0
typedef std::string DescriptorType;
#endif
#endif

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
