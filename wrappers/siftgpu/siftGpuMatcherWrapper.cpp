#include "siftGpuMatcherWrapper.h"

#ifdef WIN32
#include <Windows.h>
#else
#include <dlfcn.h>
#endif
#include <algorithm>
#include <iostream>

SiftMatchGPU* SiftGpuMatcher::initSiftMatchGpu(int count) {
	SiftMatchGPU* (*createNew)(int);
#ifndef WIN32
	void* handle = dlopen("libsiftgpu.so", RTLD_LAZY);
	if(!handle) {
		std::cerr << "Failed to open shared lib: " << dlerror() << std::endl;
		exit(0);
	}

	dlerror();

	createNew = (SiftMatchGPU* (*) (int)) dlsym(handle, "CreateNewSiftMatchGPU");
	const char* dlsym_err = dlerror();
	if(dlsym_err) {
		std::cerr << "Failed to load fun: " << dlsym_err << std::endl;
		exit(0);
	}
#else
	HINSTANCE hinstLib;
		
	hinstLib = LoadLibraryA("siftgpu.dll");

	if(!hinstLib) {
		std::cerr << "Failed to load shared lib" << std::endl;
	}

	createNew = (SiftGPU* (*) (int)) GetProcAddress(hinstLib, "CreateNewSiftGPU");

	if(!createNew) {
		std::cerr << "Failed to load function" << std::endl;
	}
#endif
	SiftMatchGPU* res = createNew(count);
	if(!(res && res->VerifyContextGL())) {
		std::cerr << "Failed to initalize SiftGPU matcher" << std::endl;
		exit(0);
	}

	return res;
}

SiftGpuMatcher::SiftGpuMatcher() {
	siftMatchGpu = initSiftMatchGpu();
}

SiftGpuMatcher::~SiftGpuMatcher() {
	delete siftMatchGpu;
}

SiftGpuMatcher::SiftGpuMatcher(const SiftGpuMatcher &matcher) {
	// TODO: check if SiftMatchGPU is stateless.
	// Note: by default __max_sift member is inaccessible
	siftMatchGpu = initSiftMatchGpu();
}

void SiftGpuMatcher::knnMatchImpl( RuntimeTypeBuffer &queryDescriptors, RuntimeTypeBuffer &trainDescriptors, std::vector<std::vector<RawMatch> >& matches, size_t K) {
	if(K != 1) {
		std::cerr << "SiftGPU matcher does not support k-NN and does not"
					 "provide matching distance" << std::endl;
	}
	if(!queryDescriptors.isValid() || !trainDescriptors.isValid()) {
		matches.clear();
		return;
	}

	assert(queryDescriptors.getType() == trainDescriptors.getType());
	assert(queryDescriptors.getType() == RuntimeBufferDataType::F32);

	matches.resize(queryDescriptors.getRows());

	for(size_t i = 0; i < queryDescriptors.getRows(); ++i) {
		matches[i].resize(0);
		matches[i].reserve(K);
	}

	size_t maxRows = std::max(queryDescriptors.getRows(), trainDescriptors.getRows());

	if(maxRows > 8192)
		siftMatchGpu->SetMaxSift(maxRows);

	int (*buffer)[2] = new int[maxRows][2];

	siftMatchGpu->SetDescriptors(0, queryDescriptors.getRows(), queryDescriptors.row<float>(0));
	siftMatchGpu->SetDescriptors(1, trainDescriptors.getRows(), trainDescriptors.row<float>(0));
	int nmatch = siftMatchGpu->GetSiftMatch(maxRows, buffer);
	for(int j = 0; j < nmatch; ++j) {
		int queryIdx = buffer[j][0];
		int trainIdx = buffer[j][1];

		matches[queryIdx].push_back(RawMatch(queryIdx, trainIdx, 0.5));
	}
	
	for(size_t i = 0; i < queryDescriptors.getRows(); ++i) {
		matches[i].resize(std::min(K, matches[i].size()));
	}


	delete[] buffer;
}

void init_siftgpu_matcher_provider() {
	DescriptorMatcherProvider::getInstance().add(new SiftGpuDescriptorMatcherProvider());
}

#define SWITCH_TYPE(str, expr) \
	if(type == #str) { \
		expr \
	}

DescriptorMatcher* SiftGpuDescriptorMatcherProvider::getDescriptorMatcher(const DescriptorType &type, const DetectorsParams &params) {
	SWITCH_TYPE(SIFTGPU, return new SiftGpuMatcher;);
	assert(false);
	return 0;
}

bool SiftGpuDescriptorMatcherProvider::provides(const DescriptorType &type) {
	SWITCH_TYPE(SIFTGPU, return true;);
	return false;
}

#undef SWITCH_TYPE



