#pragma once

#include <string>
#include "algoBase.h"
#include "imageKeyPoints.h"
#include "featureMatchingPipeline.h"


class DetectAndExtract : public AlgoBase
{
public:
    void detectAndExtract( RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors, int nMaxKeypoints );
    virtual ~DetectAndExtract() {}

protected:
    virtual void detectAndExtractImpl( RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors, int nMaxKeypoints ) = 0;
};

class DetectAndExtractProviderImpl
{
public:
    virtual DetectAndExtract* getDetector( const DetectorType &detectorType, const DescriptorType &descriptorType, const std::string &params = "" ) = 0;
    virtual bool provides( const DetectorType &detectorType, const DescriptorType &descriptorType ) = 0;

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
