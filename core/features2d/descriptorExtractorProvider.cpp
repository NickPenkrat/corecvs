#include "descriptorExtractorProvider.h"


DescriptorExtractor* DescriptorExtractorProvider::getDescriptorExtractor(const DescriptorType &type) {
	for(auto p: providers) {
		if(p->provides(type)) {
			return p->getDescriptorExtractor(type);
		}
	}
	assert(false);
	return 0;
}

void DescriptorExtractorProvider::add(DescriptorExtractorProviderImpl *provider) {
	providers.push_back(provider);
}

DescriptorExtractorProvider::~DescriptorExtractorProvider() {
	for(auto p: providers)
		delete p;
	providers.clear();
}

DescriptorExtractorProvider& DescriptorExtractorProvider::getInstance() {
	static DescriptorExtractorProvider provider;
	return provider;
}

DescriptorExtractorProvider::DescriptorExtractorProvider() {
}

void DescriptorExtractor::compute(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &buffer) {
	computeImpl(image, keyPoints, buffer);
}
