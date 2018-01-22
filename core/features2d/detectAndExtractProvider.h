#ifndef DETECTANDEXTRACTPROVIDER_H
#define DETECTANDEXTRACTPROVIDER_H

#include <string>
#include <vector>

#include "core/features2d/algoBase.h"
#include "core/features2d/imageKeyPoints.h"
#include "core/features2d/featureMatchingPipeline.h"


class DetectAndExtract : public AlgoBase
{
public:
	void detectAndExtract(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, corecvs::RuntimeTypeBuffer &descriptors, int nMaxKeypoints, void* pRemapCache);
    virtual ~DetectAndExtract() {}

protected:
	virtual void detectAndExtractImpl(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, corecvs::RuntimeTypeBuffer &descriptors, int nMaxKeypoints, void* pRemapCache) = 0;
};

class DetectAndExtractProviderImpl : public AlgoNaming
{
public:
    virtual DetectAndExtract* getDetector(const DetectorType &detectorType, const DescriptorType &descriptorType, const std::string &params = "") = 0;
    virtual bool provides(const DetectorType &detectorType, const DescriptorType &descriptorType) = 0;

    virtual ~DetectAndExtractProviderImpl() {}
};

class DetectAndExtractProvider : public AlgoCollectionNaming<DetectAndExtractProviderImpl>
{
public:
    static DetectAndExtractProvider& getInstance();
    ~DetectAndExtractProvider();

    DetectAndExtract* getDetector(const DetectorType &detectorType, const DescriptorType &descriptorType, const std::string &params = "");

private:
    DetectAndExtractProvider();
    DetectAndExtractProvider(const DetectAndExtractProvider&);
  //DetectAndExtractProvider& operator=(const DetectAndExtractProvider&);
};

#endif // DETECTANDEXTRACTPROVIDER_H
