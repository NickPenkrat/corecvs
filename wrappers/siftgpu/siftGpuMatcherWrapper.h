#ifndef SIFTGPUMATCHERWRAPPER_H
#define SIFTGPUMATCHERWRAPPER_H

#include <vector>

#include "descriptorMatcherProvider.h"

#include "SiftGPU/SiftGPU.h"



class SiftGpuMatcher : public DescriptorMatcher {
	public:
		SiftGpuMatcher();
		SiftGpuMatcher(const SiftGpuMatcher &c);
		virtual ~SiftGpuMatcher();

	protected:
		void knnMatchImpl( RuntimeTypeBuffer &query, RuntimeTypeBuffer &train, std::vector<std::vector<RawMatch> >& matches, size_t K);
	private:
		SiftMatchGPU* initSiftMatchGpu(int count = 8192);
		SiftMatchGPU* siftMatchGpu;
};

void __attribute__ ((constructor)) init_siftgpu_matcher_provider();


class SiftGpuDescriptorMatcherProvider : public DescriptorMatcherProviderImpl {
	public:
		DescriptorMatcher* getDescriptorMatcher(const DescriptorType &type, const DetectorsParams &params = DetectorsParams());
		bool provides(const DescriptorType &type);
		~SiftGpuDescriptorMatcherProvider() {}
	protected:
};

#endif
