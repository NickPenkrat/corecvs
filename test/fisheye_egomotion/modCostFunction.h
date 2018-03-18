#ifndef MODCOSTFUNCTION_H
#define MODCOSTFUNCTION_H

#include <vector>
#include <core/rectification/correspondenceList.h>
#include <core/cameracalibration/projection/omnidirectionalProjection.h>
#include <core/math/affine.h>
#include <core/function/function.h>

using namespace corecvs;

class MODCostFunctionData
{
public:
    CorrespondenceList obseravtions;
    vector<Affine3DQ> transform;
    OmnidirectionalProjection projection;
    MODCostFunctionData();

    static const int OMNIDIR_POWER = 4;
    static const int MOVE_MODEL = 7;


    int getInputSize()
    {
        return OMNIDIR_POWER + transform.size() * MOVE_MODEL;
    }

    int getOutputSize()
    {
        return obseravtions.size();
    }
};

class MODCostFunction : public FunctionArgs
{
public:
    MODCostFunctionData *data = NULL;

    MODCostFunction(MODCostFunctionData *data) :
        FunctionArgs(data->getInputSize(), data->getOutputSize()),
        data(data)
    {

    }

    virtual void operator()(const double in[], double out[]);

};


class MODCostFunctionNormalise : public FunctionArgs
{
    MODCostFunctionData *data = NULL;

    MODCostFunctionNormalise(MODCostFunctionData *data) :
        FunctionArgs(data->getInputSize(), data->getInputSize()),
        data(data)
    {
    }


    virtual void operator()(const double in[], double out[]);
};

#endif // MODCOSTFUNCTION_H
