#ifndef SIFTGPUWRAPPER_H
#define SIFTGPUWRAPPER_H

#if 0
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#endif

#include "imageKeyPoints.h"
#include "descriptorExtractorProvider.h"
#include "featureDetectorProvider.h"


#include "SiftGPU/SiftGPU.h"

class SiftGpu : public FeatureDetector, public DescriptorExtractor {
public:
	SiftGpu(double filterWidthFactor = 4.0,
			double orientationFactor = 2.0,
			double descriptorGridSize = 3.0,
			int firstOctave = 0,
			int dogLevels = 3,
			double dogThreshold = 0.02/3,
			double edgeThreshold = 10.0,
			int orientationNumber = 2);
	~SiftGpu();
	int descriptorSize() const;
	int descriptorType() const;
    
    void operator()(RuntimeTypeBuffer &img, std::vector<KeyPoint>& keypoints) const;
    
    void operator()(RuntimeTypeBuffer &img, std::vector<KeyPoint> &keypoints, RuntimeTypeBuffer &descriptors, bool computeDescriptors = false, bool useProvidedKeypoints = false) const;
protected:
	void computeImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors);
	void detectImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints);

    static SiftGPU::SiftKeypoint convert(const KeyPoint &k);
    static KeyPoint convert(const SiftGPU::SiftKeypoint &k);

    double filterWidthFactor;
    double orientationFactor;
    double descriptorGridSize;
    int firstOctave;
    int dogLevels;
    double dogThreshold;
    double edgeThreshold;
    int orientationNumber;
    // FIXME: maybe we do not this pointer at all, eg store it near the call
    mutable SiftGPU* siftGpu;
};

extern "C" {
void __attribute__ ((constructor)) __attribute__ ((used)) init_siftgpu_detector_provider();
void __attribute__ ((constructor)) __attribute__ ((used)) init_siftgpu_descriptor_provider();
}

class SiftGpuFeatureDetectorProvider : public FeatureDetectorProviderImpl {
	public:
		FeatureDetector* getFeatureDetector(const DetectorType &type, const DetectorsParams &params = DetectorsParams());
		bool provides(const DetectorType &type);
		~SiftGpuFeatureDetectorProvider() {}
	protected:
};

class SiftGpuDescriptorExtractorProvider : public DescriptorExtractorProviderImpl {
	public:
		DescriptorExtractor* getDescriptorExtractor(const DescriptorType &type, const DetectorsParams &params = DetectorsParams());
		bool provides(const DescriptorType &type);
		~SiftGpuDescriptorExtractorProvider() {}
	protected:
};

#endif
