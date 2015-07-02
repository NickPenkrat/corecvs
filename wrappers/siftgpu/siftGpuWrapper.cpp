#include "siftGpuWrapper.h"

#include <dlfcn.h>
#include <iostream>

#include "runtimeTypeBuffer.h"

#include "GL/glew.h"

SiftGpu::SiftGpu(double filterWidthFactor,
		double orientationFactor,
		double descriptorGridSize,
		int firstOctave,
		int dogLevels,
		double dogThreshold,
		double edgeThreshold,
		int orientationNumber)
	: filterWidthFactor(filterWidthFactor), orientationFactor(orientationFactor),
	descriptorGridSize(descriptorGridSize), firstOctave(firstOctave),
	dogLevels(dogLevels), dogThreshold(dogThreshold), edgeThreshold(edgeThreshold),
	orientationNumber(orientationNumber), siftGpu(0) {

		void* handle = dlopen("libsiftgpu.so", RTLD_LAZY);
		if(!handle) {
			std::cerr << "Failed to open shared lib: " << dlerror() << std::endl;
			exit(0);
		}

		dlerror();

		SiftGPU* (*createNew)(int) = (SiftGPU* (*) (int)) dlsym(handle, "CreateNewSiftGPU");
		const char* dlsym_err = dlerror();
		if(dlsym_err) {
			std::cerr << "Failed to load fun: " << dlsym_err << std::endl;
			exit(0);
		}

		siftGpu = createNew(1);
		const char *argv[] = {"-v", "0", "-fo", "-1", "-da", "-tc2", "7680", "-nomc"};
		int argc = 8;

		siftGpu->ParseParam(argc, const_cast<char**>(argv));
		int support = siftGpu->CreateContextGL();
		if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
			std::cerr << support << " siftgpu is unsupported" << std::endl;
			exit(0);
		}
	}

void SiftGpu::computeImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors) {
	(*this)(image, keyPoints, descriptors, true, true);
}

void SiftGpu::detectImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints) {
	RuntimeTypeBuffer buffer;
	(*this)(image, keyPoints,buffer);
}

int SiftGpu::descriptorSize() const { 
	return 128;
}

int SiftGpu::descriptorType() const { 
	return CV_32F;
}

void SiftGpu::operator()(RuntimeTypeBuffer &img, std::vector<KeyPoint>& keypoints) const {
	RuntimeTypeBuffer buffer;
	(*this)(img, keypoints, buffer);
}

SiftGPU::SiftKeypoint SiftGpu::convert(const KeyPoint &k) {
	//TODO: check what orientation and size really represent
	return { k.x, k.y, k.size, k.angle };
}

KeyPoint SiftGpu::convert(const SiftGPU::SiftKeypoint &k) {
	return KeyPoint(k.x, k.y, k.s, k.o);
}

void SiftGpu::operator()(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keypoints, RuntimeTypeBuffer &descriptors, bool computeDescriptors, bool useProvidedKeypoints) const {
	if(image.getType() != RuntimeBufferDataType::U8 || !image.isValid()) {
		std::cerr << __LINE__ << "Invalid image type" << std::endl;
	}
	
	std::vector<SiftGPU::SiftKeypoint> keypoints_sgpu;
	if( useProvidedKeypoints ) {
		keypoints_sgpu.reserve(keypoints.size());
		for(auto kp: keypoints)
			keypoints_sgpu.push_back(SiftGpu::convert(kp));
		siftGpu->SetKeypointList(keypoints_sgpu.size(), &keypoints_sgpu[0]);
		siftGpu->RunSIFT(image.getCols(), image.getRows(), image.row<unsigned char>(0), GL_LUMINANCE, GL_UNSIGNED_BYTE);
	} else {
		siftGpu->RunSIFT(image.getCols(), image.getRows(), image.row<unsigned char>(0), GL_LUMINANCE, GL_UNSIGNED_BYTE);
		int num = siftGpu->GetFeatureNum();
		keypoints_sgpu.resize(num);
		siftGpu->GetFeatureVector(&keypoints_sgpu[0], 0);
		keypoints.reserve(num);
		for(auto k: keypoints_sgpu)
			keypoints.push_back(SiftGpu::convert(k));
	}

	if(computeDescriptors) {
		descriptors = RuntimeTypeBuffer(keypoints.size(), descriptorSize(), RuntimeBufferDataType::F32);
		siftGpu->GetFeatureVector(0, descriptors.row<float>(0));
	}
}

SiftGpu::~SiftGpu() {
	delete siftGpu;
}

#define SWITCH_TYPE(str, expr) \
	if(type == #str) { \
		expr \
	}

DescriptorExtractor* SiftGpuDescriptorExtractorProvider::getDescriptorExtractor(const DescriptorType &type, const DetectorsParams &params) {
	auto siftParams = params.siftParams;
	auto surfParams = params.surfParams;
	auto starParams = params.starParams;
	auto fastParams = params.fastParams;
	auto briskParams = params.briskParams;
	auto orbParams = params.orbParams;
	SWITCH_TYPE(SIFTGPU, 
		return new SiftGpu();)
	return 0;
}

bool SiftGpuDescriptorExtractorProvider::provides(const DescriptorType &type) {
	SWITCH_TYPE(SIFTGPU, return true;);
	return false;
}


FeatureDetector* SiftGpuFeatureDetectorProvider::getFeatureDetector(const DetectorType &type, const DetectorsParams &params) {
	auto siftParams = params.siftParams;
	auto surfParams = params.surfParams;
	auto starParams = params.starParams;
	auto fastParams = params.fastParams;
	auto briskParams = params.briskParams;
	auto orbParams = params.orbParams;
	SWITCH_TYPE(SIFTGPU, 
		return new SiftGpu();)
	return 0;
}

bool SiftGpuFeatureDetectorProvider::provides(const DetectorType &type) {
	SWITCH_TYPE(SIFTGPU, return true;);
	return false;
}

void init_siftgpu_detector_provider() {
	FeatureDetectorProvider::getInstance().add(new SiftGpuFeatureDetectorProvider);
}
void init_siftgpu_descriptor_provider() {
	DescriptorExtractorProvider::getInstance().add(new SiftGpuDescriptorExtractorProvider);
}

#undef SWITCH_TYPE
