#include "descriptorMatcherProvider.h"

#include <cassert>

DescriptorMatcher* DescriptorMatcherProvider::getMatcher(const DescriptorType &type) {
    for(std::vector<DescriptorMatcherProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p) {
        if((*p)->provides(type)) {
            return (*p)->getDescriptorMatcher(type);
		}
	}
	assert(false);
	return 0;
}

void DescriptorMatcher::knnMatch(RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch>> &matches, size_t K) {
	knnMatchImpl(query, train, matches, K);
}

DescriptorMatcherProvider::~DescriptorMatcherProvider() {
    for(std::vector<DescriptorMatcherProviderImpl*>::iterator p = providers.begin(); p != providers.end(); ++p) {
        delete *p;
    }
	providers.clear();
}

void DescriptorMatcherProvider::add(DescriptorMatcherProviderImpl *provider) {
	providers.push_back(provider);
}

DescriptorMatcherProvider& DescriptorMatcherProvider::getInstance() {
	static DescriptorMatcherProvider provider;
	return provider;
}

DescriptorMatcherProvider::DescriptorMatcherProvider() {
}
