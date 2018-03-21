#ifndef MODCOSTFUNCTION_H
#define MODCOSTFUNCTION_H

#include <vector>
#include "core/rectification/correspondenceList.h"
#include "core/cameracalibration/projection/omnidirectionalProjection.h"
#include "core/math/affine.h"
#include "core/function/function.h"
#include "core/math/levenmarq.h"

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

class MODModel
{
public:
    vector<Affine3DQ> transform;
    vector<double> projectionN;
    MODCostFunctionData *context = NULL;

    double getCost(const Correspondence &corr);

    bool fits(const Correspondence &data, double fitTreshold)
    {
        return (getCost(data) < fitTreshold);
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
public:
    MODCostFunctionData *data = NULL;

    MODCostFunctionNormalise(MODCostFunctionData *data) :
        FunctionArgs(data->getInputSize(), data->getInputSize()),
        data(data)
    {
    }


    virtual void operator()(const double in[], double out[]);
};



class MODJointOptimisation
{
public:
    typedef Correspondence  SampleType;
    typedef MODModel        ModelType;

    MODCostFunctionData *baseData = NULL;

    MODJointOptimisation() {}

    static vector<MODModel> getModels(const vector<Correspondence *> &samples, MODJointOptimisation *context)
    {
        vector<MODModel> toReturn;

        if (context == NULL || context->baseData == NULL)
            return toReturn;

        /** Simplified vertion with less observations **/
        MODCostFunctionData data;

        data.projection = context->baseData->projection;
        data.transform  = context->baseData->transform;

        data.obseravtions.reserve(samples.size());
        for (size_t i = 0; i < samples.size(); i++)
        {
            data.obseravtions.push_back(*samples[i]);
        }

        MODCostFunction          cost(&data);
        MODCostFunctionNormalise normalise(&data);

        LevenbergMarquardt LMfit;
        LMfit.f = &cost;
        LMfit.normalisation = &normalise;
        LMfit.maxIterations = 50;
        LMfit.trace = false;
        LMfit.traceProgress = false;
        vector<double> input;
        vector<double> output;
        vector<double> result;


        input.resize (cost.inputs);
        output.resize(cost.outputs, 0);

        for (int i = 0; i < MODCostFunctionData::OMNIDIR_POWER; i++)
            input[i] = data.projection.mN[i];

        for (size_t i = 0; i < data.transform.size(); i++)
        {
            Affine3DQ guess = data.transform[i];
            guess.shift.storeTo(&input[i * MODCostFunctionData::MOVE_MODEL + MODCostFunctionData::OMNIDIR_POWER]);
            guess.rotor.storeTo(&input[i * MODCostFunctionData::MOVE_MODEL + MODCostFunctionData::OMNIDIR_POWER + 3]);
        }

        result = LMfit.fit(input, output);

        MODModel model;
        model.context = context->baseData;
        model.transform.resize(data.transform.size());

        for (size_t i = 0; i < data.transform.size(); i++)
        {
            model.transform[i].shift.loadFrom(&result[MODCostFunctionData::OMNIDIR_POWER]);
            model.transform[i].rotor.loadFrom(&result[MODCostFunctionData::OMNIDIR_POWER + 3]);
        }

        for (int i = 0; i < MODCostFunctionData::OMNIDIR_POWER; i++)
        {
            model.projectionN.push_back(result[i]);
        }
        cout << result;
        toReturn.push_back(model);
        return toReturn;

    }


};


#endif // MODCOSTFUNCTION_H
