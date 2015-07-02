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
#if 0
void SiftGpuMatcher::knnMatchImpl( const cv::Mat& queryDescriptors, std::vector<std::vector<cv::DMatch> >& matches, int knn, const std::vector<cv::Mat>& masks, bool compactResult ) {
	if(knn != 1) {
		std::cerr << "SiftGPU matcher does not support k-NN and does not"
					 "provide matching distance" << std::endl;
	}
	if(queryDescriptors.empty() || trainDescCollection.empty()) {
		matches.clear();
		return;
	}

	assert(queryDescriptors.type() == trainDescCollection[0].type());
	assert(queryDescriptors.type() == CV_32F);

	matches.resize(queryDescriptors.rows);

	size_t imgCount = trainDescCollection.size();

	for(size_t i = 0; i < (size_t)queryDescriptors.rows; ++i) {
		matches[i].resize(0);
		matches[i].reserve(knn);
	}

	size_t maxRows = queryDescriptors.rows;
	for(size_t i = 0; i < imgCount; ++i)
		maxRows = std::max(maxRows, (size_t)trainDescCollection[i].rows);

	if(maxRows > 8192)
		siftMatchGpu->SetMaxSift(maxRows);

	int (*buffer)[2] = new int[maxRows][2];

	siftMatchGpu->SetDescriptors(0, queryDescriptors.rows, (float*)queryDescriptors.data);
	for(size_t i = 0; i < imgCount; ++i) {
		siftMatchGpu->SetDescriptors(1, trainDescCollection[i].rows, (float*)trainDescCollection[i].data);
		int nmatch = siftMatchGpu->GetSiftMatch(maxRows, buffer);
		for(int j = 0; j < nmatch; ++j) {
			int queryIdx = buffer[j][0];
			int trainIdx = buffer[j][1];

			matches[queryIdx].push_back(cv::DMatch(queryIdx, trainIdx, i, 0.5));
		}
	}
	size_t idx = 0;
	for(size_t i = 0; i < (size_t)queryDescriptors.rows; ++i) {
		matches[i].resize(std::min(knn, (int)matches[i].size()));
		if(matches[i].size() && compactResult) {
			matches[idx++] = matches[i];
		}
	}
	if(compactResult) {
		matches.resize(idx);
	}

	delete[] buffer;
}

void SiftGpuMatcher::radiusMatchImpl( const cv::Mat& queryDescriptors, std::vector<std::vector<cv::DMatch> >& matches, float maxDistance,
				const std::vector<cv::Mat>& masks, bool compactResult) {

	if(queryDescriptors.empty() || trainDescCollection.empty()) {
		matches.clear();
		return;
	}

	assert(queryDescriptors.type() == trainDescCollection[0].type());
	assert(queryDescriptors.type() == CV_32F);

	matches.reserve(queryDescriptors.rows);

	size_t imgCount = trainDescCollection.size();

	for(size_t i = 0; i < (size_t)queryDescriptors.rows; ++i) {
		matches[i].resize(0);
	}

	size_t maxRows = queryDescriptors.rows;
	for(size_t i = 0; i < imgCount; ++i)
		maxRows = std::max(maxRows, (size_t)trainDescCollection[i].rows);

	if(maxRows > 8192)
		siftMatchGpu->SetMaxSift(maxRows);

	int (*buffer)[2] = new int[maxRows][2];

	siftMatchGpu->SetDescriptors(0, queryDescriptors.rows, (float*)queryDescriptors.data);
	for(size_t i = 0; i < imgCount; ++i) {
		siftMatchGpu->SetDescriptors(1, trainDescCollection[i].rows, (float*)trainDescCollection[i].data);
		int nmatch = siftMatchGpu->GetSiftMatch(maxRows, buffer, maxDistance);
		for(int j = 0; j < nmatch; ++j) {
			int queryIdx = buffer[j][0];
			int trainIdx = buffer[j][1];

			matches[queryIdx].push_back(cv::DMatch(queryIdx, trainIdx, i, 0.5));
		}
	}
	size_t idx = 0;
	for(size_t i = 0; i < (size_t)queryDescriptors.rows; ++i) {
		if(matches[i].size() && compactResult) {
			matches[idx++] = matches[i];
		}
	}
	if(compactResult) {
		matches.resize(idx);
	}

	delete[] buffer;
}
#endif

void init_siftgpu_matcher_provider() {
	DescriptorMatcherProvider::getInstance().add(new SiftGpuDescriptorMatcherProvider());
}

#define SWITCH_TYPE(str, expr) \
	if(type == #str) { \
		expr \
	}

DescriptorMatcher* SiftGpuDescriptorMatcherProvider::getDescriptorMatcher(const DescriptorType &type, const DetectorsParams &params) {
#if 0
	auto siftParams = params.siftParams;
	auto surfParams = params.surfParams;
	auto starParams = params.starParams;
	auto fastParams = params.fastParams;
	auto briskParams = params.briskParams;
	auto orbParams = params.orbParams;
#endif
	SWITCH_TYPE(SIFTGPU, return new SiftGpuMatcher;);
	assert(false);
	return 0;
}

bool SiftGpuDescriptorMatcherProvider::provides(const DescriptorType &type) {
	SWITCH_TYPE(SIFTGPU, return true;);
	return false;
}

#undef SWITCH_TYPE



