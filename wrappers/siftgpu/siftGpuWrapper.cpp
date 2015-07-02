#include "siftGpuWrapper.h"

#include <dlfcn.h>
#include <iostream>

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

void SiftGpu::computeImpl(DescriptorBuffer &image, std::vector<KeyPoint> &keyPoints, DescriptorBuffer &descriptors) {
	(*this)(image, keyPoints, descriptors, true, true);
}

void SiftGpu::detectImpl(DescriptorBuffer &image, std::vector<KeyPoint> &keyPoints) {
	DescriptorBuffer buffer;
	(*this)(image, keyPoints,buffer);
}

#if 0
void SiftGpu::detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const {
	(*this)(image, mask, keypoints, cv::noArray());
}


void SiftGpu::computeImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors ) const {
	(*this)(image, cv::Mat(), keypoints, descriptors, true);
}
#endif


int SiftGpu::descriptorSize() const { 
	return 128;
}

int SiftGpu::descriptorType() const { 
	return CV_32F;
}

void SiftGpu::operator()(DescriptorBuffer &img, std::vector<KeyPoint>& keypoints) const {
	DescriptorBuffer buffer;
	(*this)(img, keypoints, buffer);
}

#if 0
void SiftGpu::operator()(cv::InputArray img, cv::InputArray mask,
		std::vector<cv::KeyPoint>& keypoints) const {
	(*this)(img, mask, keypoints, cv::noArray());
}
#endif

SiftGPU::SiftKeypoint SiftGpu::convert(const KeyPoint &k) {
	//TODO: check what orientation and size really represent
	return { k.x, k.y, k.size, k.angle };
}

KeyPoint SiftGpu::convert(const SiftGPU::SiftKeypoint &k) {
	return KeyPoint(k.x, k.y, k.s, k.o);
}

void SiftGpu::operator()(DescriptorBuffer &image, std::vector<KeyPoint> &keypoints, DescriptorBuffer &descriptors, bool computeDescriptors, bool useProvidedKeypoints) const {
	if(image.getType() != BufferDataType::U8 || !image.isValid()) {
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
		std::cerr << "Running SIFTGPU" << std::endl;
		siftGpu->RunSIFT(image.getCols(), image.getRows(), image.row<unsigned char>(0), GL_LUMINANCE, GL_UNSIGNED_BYTE);
		int num = siftGpu->GetFeatureNum();
		keypoints_sgpu.resize(num);
		siftGpu->GetFeatureVector(&keypoints_sgpu[0], 0);
		keypoints.reserve(num);
		for(auto k: keypoints_sgpu)
			keypoints.push_back(SiftGpu::convert(k));
	}

	if(computeDescriptors) {
		descriptors = DescriptorBuffer(keypoints.size(), descriptorSize(), BufferDataType::F32);
#if 0
		descriptors.create(keypoints.size(), descriptorSize(), descriptorType());
		cv::Mat _descriptors = descriptors.getMat();
#endif
		siftGpu->GetFeatureVector(0, descriptors.row<float>(0));
	}
}


#if 0
void SiftGpu::operator()(cv::InputArray img, cv::InputArray _mask,
		std::vector<cv::KeyPoint>& keypoints,
		cv::OutputArray descriptors,
		bool useProvidedKeypoints) const {
	
	cv::Mat image = img.getMat(), mask = _mask.getMat();

	if( image.empty() || image.depth() != CV_8U) {
		std::cerr << __LINE__ << " : Invalid image depth (!=CV_8U)" << std::endl;
		exit(0);
	}

	if( !mask.empty() && mask.type() != CV_8UC1) {
		std::cerr << __LINE__ << " : Invalid image depth (!=CV_8U)" << std::endl;
		exit(0);
	}
	
	std::vector<SiftGPU::SiftKeypoint> keypoints_sgpu;
	if( useProvidedKeypoints ) {
		keypoints_sgpu.reserve(keypoints.size());
		for(auto kp: keypoints)
			keypoints_sgpu.push_back(SiftGpu::convert(kp));
		siftGpu->SetKeypointList(keypoints_sgpu.size(), &keypoints_sgpu[0]);
		siftGpu->RunSIFT(image.cols, image.rows, (unsigned char*)image.data, GL_LUMINANCE, GL_UNSIGNED_BYTE);
	} else {
		std::cerr << "Running SIFTGPU" << std::endl;
		siftGpu->RunSIFT(image.cols, image.rows, (unsigned char*)image.data, GL_LUMINANCE, GL_UNSIGNED_BYTE);
		int num = siftGpu->GetFeatureNum();
		keypoints_sgpu.resize(num);
		siftGpu->GetFeatureVector(&keypoints_sgpu[0], 0);
		keypoints.reserve(num);
		for(auto k: keypoints_sgpu)
			keypoints.push_back(SiftGpu::convert(k));
	}

	if(descriptors.needed()) {
		descriptors.create(keypoints.size(), descriptorSize(), descriptorType());
		cv::Mat _descriptors = descriptors.getMat();

		siftGpu->GetFeatureVector(0, (float*)_descriptors.data);
	}
}
#endif

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
