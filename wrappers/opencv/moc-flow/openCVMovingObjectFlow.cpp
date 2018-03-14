#include "core/xml/generated/rgbColorParameters.h"
#include "openCVMovingObjectFlow.h"

using namespace corecvs;

OpenCVMovingObjectFlow::OpenCVMovingObjectFlow()
{

}

int OpenCVMovingObjectFlow::beginFrame()
{
    SYNC_PRINT(("OpenCVMovingObjectFlow::beginFrame():called/n"));
    return 0;
}

int OpenCVMovingObjectFlow::clean(int mask)
{
    SYNC_PRINT(("OpenCVMovingObjectFlow::clean(int mask):called/n"));
    return 0;
}

int OpenCVMovingObjectFlow::setFrameG12(Processor6D::FrameNames frameType, G12Buffer *frame)
{
    SYNC_PRINT(("OpenCVMovingObjectFlow::setFrameG12():called/n"));
    return 0;
}

int OpenCVMovingObjectFlow::setFrameRGB24(Processor6D::FrameNames frameType, RGB24Buffer *frame)
{

    SYNC_PRINT(("OpenCVMovingObjectFlow::setFrameRGB24():called/n"));
    return 0;
}

int OpenCVMovingObjectFlow::setDisparityBufferS16(Processor6D::FrameNames frameType, FlowBuffer *frame)
{
    SYNC_PRINT(("OpenCVMovingObjectFlow::setDisparityBufferS16():called/n"));
    return 0;
}

int OpenCVMovingObjectFlow::setStats(corecvs::Statistics *stats)
{
    this->stats = stats;
    return 0;
}

/* Main computation is done here */
int OpenCVMovingObjectFlow::endFrame()
{

    delete_safe(meshFlow);
    meshFlow = new MeshFlow;
    meshFlow->stats = stats;

    meshFlow->init(
        first.rows,
        first.cols,
        params.gridRows(),
        params.gridColumns(),
        params.ransacGridRows(),
        params.ransacGridColumns(),
        params.medianFilterSizeH(),
        params.medianFilterSizeW(),
        params.maxFeatureNumber(),
        params.featureTreshold()
                );

    meshFlow->computeMeshFlow(first, second);

    return 0;
}

std::map<std::string, DynamicObject> OpenCVMovingObjectFlow::getParameters()
{
    std::map<std::string, DynamicObject> result;
    result.emplace("main", DynamicObject(&params));

    return result;
}

bool OpenCVMovingObjectFlow::setParameters(std::string name, const DynamicObject &param)
{
    if (name == "main") {
        param.copyTo(&params);
        cout << params;
        return true;
    }
    return false;
}

int OpenCVMovingObjectFlow::setParameteri(int parameterName, int parameterValue)
{
    CORE_UNUSED(parameterName);
    CORE_UNUSED(parameterValue);
    return 0;
}

int OpenCVMovingObjectFlow::requestResultsi(int parameterName)
{
    CORE_UNUSED(parameterName);
    return 0;
}

FlowBuffer *OpenCVMovingObjectFlow::getFlow()
{

    return result;
}

corecvs::FlowBuffer *OpenCVMovingObjectFlow::getStereo()
{
    return NULL;
}

CorrespondenceList *OpenCVMovingObjectFlow::getFlowList()
{
    if (meshFlow == NULL)
        return NULL;

    CorrespondenceList* list = new CorrespondenceList;

    for (size_t i = 0; i < meshFlow->feat_cur_.size(); i++)
    {
        Point2f &cvData0  = meshFlow->feat_prev_[i];
        Point2f &cvData1  = meshFlow->feat_cur_ [i];

        Vector2dd data0(cvData0.x, cvData0.y);
        Vector2dd data1(cvData1.x, cvData1.y);
        list->push_back(Correspondence(data0, data1));

    }
    return list;
}

int OpenCVMovingObjectFlow::getError(std::string *errorString)
{
    CORE_UNUSED(errorString);
    return 0;
}



