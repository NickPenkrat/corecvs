#ifndef MODCOSTFUNCTION_H
#define MODCOSTFUNCTION_H

#include <vector>
#include "core/rectification/correspondenceList.h"
#include "core/cameracalibration/projection/omnidirectionalProjection.h"
#include "core/math/affine.h"
#include "core/function/function.h"
#include "core/math/levenmarq.h"

using namespace corecvs;


class MODPointsCostFunctionData
{
public:
    CorrespondenceList obseravtions;
    vector<Affine3DQ> transform;
    vector<Vector3dd> points;
    OmnidirectionalProjection projection;

    MODPointsCostFunctionData();

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

    static Affine3DQ load (const double *in);
    static void store(const Affine3DQ &input, double *out);


};

class MODPointsModel
{
public:
    vector<Affine3DQ> transform;
    vector<double> projectionN;
    MODPointsCostFunctionData *context = NULL;

    double getCost(const Correspondence &corr);

    bool fits(const Correspondence &data, double fitTreshold)
    {
        return (getCost(data) < fitTreshold);
    }
};


class MODPointsCostFunction : public FunctionArgs
{
public:
    MODPointsCostFunctionData *data = NULL;

    MODPointsCostFunction(MODPointsCostFunctionData *data) :
        FunctionArgs(data->getInputSize(), data->getOutputSize()),
        data(data)
    {

    }

    static double cost(const Affine3DQ &T, const CameraProjection &projection, const Correspondence &corr);

    virtual void operator()(const double in[], double out[]);

};


class MODPointsCostFunctionNormalise : public FunctionArgs
{
public:
    MODPointsCostFunctionData *data = NULL;

    MODPointsCostFunctionNormalise(MODPointsCostFunctionData *data) :
        FunctionArgs(data->getInputSize(), data->getInputSize()),
        data(data)
    {
    }


    virtual void operator()(const double in[], double out[]);
};



class MODPointsJointOptimisation
{
public:
    typedef Correspondence  SampleType;
    typedef MODPointsModel        ModelType;

    MODPointsCostFunctionData *baseData = NULL;

    MODPointsJointOptimisation() {}

    static vector<MODPointsModel> getModels(const vector<Correspondence *> &samples, MODPointsJointOptimisation *context)
    {
        vector<MODPointsModel> toReturn;

        if (context == NULL || context->baseData == NULL)
            return toReturn;

        /** Simplified vertion with less observations **/
        MODPointsCostFunctionData data;

        data.projection = context->baseData->projection;
        data.transform  = context->baseData->transform;

        data.obseravtions.reserve(samples.size());
        for (size_t i = 0; i < samples.size(); i++)
        {
            data.obseravtions.push_back(*samples[i]);
        }

        MODPointsCostFunction          cost(&data);
        MODPointsCostFunctionNormalise normalise(&data);

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

        for (int i = 0; i < MODPointsCostFunctionData::OMNIDIR_POWER; i++)
            input[i] = data.projection.mN[i];

        for (size_t i = 0; i < data.transform.size(); i++)
        {
            Affine3DQ guess = data.transform[i];
            guess.shift.storeTo(&input[i * MODPointsCostFunctionData::MOVE_MODEL + MODPointsCostFunctionData::OMNIDIR_POWER]);
            guess.rotor.storeTo(&input[i * MODPointsCostFunctionData::MOVE_MODEL + MODPointsCostFunctionData::OMNIDIR_POWER + 3]);
        }

        result = LMfit.fit(input, output);

        MODPointsModel model;
        model.context = context->baseData;
        model.transform.resize(data.transform.size());

        for (size_t i = 0; i < data.transform.size(); i++)
        {
            model.transform[i].shift.loadFrom(&result[MODPointsCostFunctionData::OMNIDIR_POWER]);
            model.transform[i].rotor.loadFrom(&result[MODPointsCostFunctionData::OMNIDIR_POWER + 3]);
        }

        for (int i = 0; i < MODPointsCostFunctionData::OMNIDIR_POWER; i++)
        {
            model.projectionN.push_back(result[i]);
        }
        //cout << result;
        toReturn.push_back(model);
        return toReturn;

    }


};


#endif // MODCOSTFUNCTION_H
