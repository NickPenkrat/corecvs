#include "descriptorMatcherProvider.h"

#include <cassert>

DescriptorMatcher* DescriptorMatcherProvider::getMatcher(const DescriptorType &type) {
	for(auto p: providers) {
		if(p->provides(type)) {
			return p->getDescriptorMatcher(type);
		}
	}
	assert(false);
	return 0;
}

void DescriptorMatcher::knnMatch(RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch>> &matches, size_t K) {
	knnMatchImpl(query, train, matches, K);
}

DescriptorMatcherProvider::~DescriptorMatcherProvider() {
	for(auto p: providers)
		delete p;
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
