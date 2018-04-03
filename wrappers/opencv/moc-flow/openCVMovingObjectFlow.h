#ifndef OPENCVMOVINGOBJECTFLOW_H
#define OPENCVMOVINGOBJECTFLOW_H

#include "core/stereointerface/processor6D.h"
#include "core/stats/calculationStats.h"
#include "xml/generated/meshFlowDrawParameters.h"
#include "meshFlowAlgo.h"


#include <math.h>
#include <numeric>
#include <time.h>

#ifdef WITH_OPENCV_3X
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#else
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

class OpenCVMovingObjectFlow : public corecvs::Processor6D
{
public:
    OpenCVMovingObjectFlow();
    MeshFlowDrawParameters params;

    corecvs::Statistics *stats = NULL;


    cv::Mat first;
    cv::Mat second;

    corecvs::FlowBuffer *result = NULL;

    MeshFlow *meshFlow = NULL;

    // Processor6D interface
public:
    virtual int beginFrame() override;

    virtual int reset() override;
    virtual int clean(int mask) override;

    virtual int setFrameG12  (FrameNames frameType, corecvs::G12Buffer   *frame) override;
    virtual int setFrameRGB24(FrameNames frameType, corecvs::RGB24Buffer *frame) override;

    virtual int setDisparityBufferS16(FrameNames frameType, corecvs::FlowBuffer *frame) override;

    virtual int setStats(corecvs::Statistics *stats) override;

    virtual int endFrame() override;

    virtual std::map<std::string, corecvs::DynamicObject> getParameters() override;
    virtual bool setParameters(std::string name, const corecvs::DynamicObject &param) override;

    virtual int setParameteri(int parameterName, int parameterValue) override;
    virtual int requestResultsi(int parameterName) override;

    virtual corecvs::FlowBuffer *getFlow() override;
    virtual corecvs::FlowBuffer *getStereo() override;
    virtual corecvs::CorrespondenceList *getFlowList() override;


    virtual int getError(std::string *errorString) override;

};

class OpenCVMovingObjectFlowImplFactory : public corecvs::Processor6DFactory {
public:
    virtual corecvs::Processor6D *getProcessor() override;

    virtual std::string getName() override
    {
        return "MeshFlow";
    }

};





#endif // OPENCVMOVINGOBJECTFLOW_H
