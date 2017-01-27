#pragma once

#include <string>
#include "algoBase.h"
#include "imageKeyPoints.h"
#include "featureMatchingPipeline.h"


class DetectAndExtract : public AlgoBase
{
public:
	void detectAndExtract(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, corecvs::RuntimeTypeBuffer &descriptors, int nMaxKeypoints, void* pRemapCache);
    virtual ~DetectAndExtract() {}

protected:
	virtual void detectAndExtractImpl(corecvs::RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, corecvs::RuntimeTypeBuffer &descriptors, int nMaxKeypoints, void* pRemapCache) = 0;
};

class DetectAndExtractProviderImpl
{
public:
    virtual DetectAndExtract* getDetector( const DetectorType &detectorType, const DescriptorType &descriptorType, const std::string &params = "" ) = 0;
    virtual bool provides( const DetectorType &detectorType, const DescriptorType &descriptorType ) = 0;
    virtual std::string name() { return "unknown"; }

    virtual ~DetectAndExtractProviderImpl() {}
};

class DetectAndExtractProvider
{
public:
    static DetectAndExtractProvider& getInstance();
    ~DetectAndExtractProvider();

    void add( DetectAndExtractProviderImpl *provider );
    DetectAndExtract* getDetector( const DetectorType &detectorType, const DescriptorType &descriptorType, const std::string &params = "" );

private:
    DetectAndExtractProvider();
    DetectAndExtractProvider( const DetectAndExtractProvider& );
    DetectAndExtractProvider& operator=( const DetectAndExtractProvider& );

    std::vector<DetectAndExtractProviderImpl*> providers;
};
