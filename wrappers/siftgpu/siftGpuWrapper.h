#ifndef SIFTGPUWRAPPER_H
#define SIFTGPUWRAPPER_H

#include "core/utils/global.h"

#include "core/buffers/runtimeTypeBuffer.h"
#include "core/features2d/imageKeyPoints.h"
#include "core/features2d/descriptorExtractorProvider.h"
#include "core/features2d/featureDetectorProvider.h"

#include "SiftGPU/src/SiftGPU/SiftGPU.h"

class SiftGpu : public FeatureDetector, public DescriptorExtractor
{
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

	// TODO: add meaningful implementation for converting params to siftgpu's argc/argv
	double getProperty(const std::string &name) const
	{
        CORE_UNUSED(name);
        return 0.0;
    }
	void setProperty(const std::string &name, const double &value)
	{
        CORE_UNUSED(name);
        CORE_UNUSED(value);
    }
    bool isParallelable() { return false; }

    void operator()(corecvs::RuntimeTypeBuffer &img, std::vector<KeyPoint>& keypoints) const;
    void operator()(corecvs::RuntimeTypeBuffer &img, std::vector<KeyPoint> &keypoints, corecvs::RuntimeTypeBuffer &descriptors, bool computeDescriptors = false, bool useProvidedKeypoints = false) const;

protected:
    void computeImpl(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, corecvs::RuntimeTypeBuffer &descriptors, void*);
    void detectImpl(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, int, void*);

	static SiftGPU::SiftKeypoint convert(const KeyPoint &k);
	static KeyPoint convert(const SiftGPU::SiftKeypoint &k);

	double filterWidthFactor;
	double orientationFactor;
	double descriptorGridSize;
	int    firstOctave;
	int    dogLevels;
	double dogThreshold;
	double edgeThreshold;
	int    orientationNumber;
	// FIXME: maybe we do not this pointer at all, eg store it near the call
	SiftGPU* siftGpu;
};

extern "C"
{
	void init_siftgpu_detector_provider();
	void init_siftgpu_descriptor_provider();
}

class SiftGpuFeatureDetectorProvider : public FeatureDetectorProviderImpl
{
public:
	FeatureDetector* getFeatureDetector(const DetectorType &type, const std::string& params = "");
    virtual bool provides(const DetectorType &type) override;
    virtual std::string name()  override {return "SiftGpu"; }
    virtual std::vector<std::string> provideHints();
	~SiftGpuFeatureDetectorProvider() {}
};

class SiftGpuDescriptorExtractorProvider : public DescriptorExtractorProviderImpl
{
public:
	DescriptorExtractor* getDescriptorExtractor(const DescriptorType &type, const std::string& params = "");
    virtual bool provides(const DescriptorType &type);
    virtual std::string name() override {return "SiftGpu";}
	~SiftGpuDescriptorExtractorProvider() {}
};

#endif // SIFTGPUWRAPPER_H
