#include "lmDistortionSolver.h"
#include "camerasCalibration/camerasCalibrationFunc.h"
#include "levenmarq.h"
#include "log.h"
#include "anglePointsFunction.h"
#include "distPointsFunction.h"

LMDistortionSolver::LMDistortionSolver()
{
}


RadialCorrection LMDistortionSolver::solve()
{
    vector<double> values;
    vector<Vector3dd> arguments;

    for (unsigned i = 0; i < list->size(); i++ )
    {
        PointObservation observation = list->operator [](i);
        values.push_back(observation.u());
        values.push_back(observation.v());
        arguments.push_back(observation.point);
    }
    CamerasCalibrationFunc func(&arguments, initialCenter);
    vector<double> bestParams;

    double params[] = {0, 0, 0, 0, -5.09, 11.1, -30.4, 35, 0.12, 0.04, -4.81, 14.6, -11.2, 11.2};
    vector<double> parameters(params, params + CORE_COUNT_OF(params));
    LevenbergMarquardt LMTransform(10000, 4, 1.2);
    LMTransform.f = &func;
    func.setIsDistored(false);
    bestParams = LMTransform.fit(parameters,values);
    func.setIsDistored(true);
    bestParams = LMTransform.fit(bestParams, values);
    vector<double> polynomKoeff;
    polynomKoeff.push_back(bestParams.at(0));
    polynomKoeff.push_back(bestParams.at(1));
    //    LensDistortionModelParameters lenCorrectParams(polynomKoeff, bestParams.at(2), bestParams.at(3), center);

    LensDistortionModelParameters lenCorrectParams(
                initialCenter.x(),
                initialCenter.y(),
                bestParams.at(2), bestParams.at(3),
                polynomKoeff,
                1.0,
                1.0,
                initialCenter.l2Metric());


    return RadialCorrection(lenCorrectParams);
 }


LMLinesDistortionSolver::LMLinesDistortionSolver()
{
    for (int costType = 0; costType < LineDistortionEstimatorCost::LINE_DISTORTION_ESTIMATOR_COST_LAST; costType++)
    {
        costs[costType] = 0;
    }
}

RadialCorrection LMLinesDistortionSolver::solve()
{
    lineList->print();
    cout << "Parameters:" << parameters << std::endl;

    vector<vector<Vector2dd> > straights = lineList->getLines();
    L_INFO_P("Starting distortion calibration on %d lines", straights.size());

    RadialCorrection correction(LensDistortionModelParameters(
       initialCenter.x(),
       initialCenter.y(),
       0.0 ,0.0,
       vector<double>(parameters.polinomDegree()),
       1.0,
       1.0,
       initialCenter.l2Metric()
    ));

    ModelToRadialCorrection modelFactory(
        correction,
        parameters.estimateCenter(),
        parameters.estimateTangent(),
        parameters.polinomDegree(),
        parameters.evenPowersOnly()
    );

    FunctionArgs *costFuntion = NULL;
    if (parameters.costAlgorithm() == LineDistortionEstimatorCost::JOINT_ANGLE_COST) {
        costFuntion = new AnglePointsFunction (straights, modelFactory);
    } else {
        costFuntion = new DistPointsFunction  (straights, modelFactory);
    }

    LevenbergMarquardt straightLevMarq(parameters.iterationNumber(), 2, 1.5);
    straightLevMarq.f = costFuntion;

    /* First aproximation is zero vector */
    vector<double> first(costFuntion->inputs, 0);
    modelFactory.getModel(correction, &first[0]);

    vector<double> value(costFuntion->outputs, 0);
    vector<double> straightParams = straightLevMarq.fit(first, value);

    L_INFO_P("Ending distortion calibration");
//    updateScore();
    return modelFactory.getRadial(&straightParams[0]);

}

void LMLinesDistortionSolver::computeCosts(const RadialCorrection &correction, bool updatePoints)
{
    vector<vector<Vector2dd> > straights = lineList->getLines();

    ModelToRadialCorrection modelFactory(
        correction,
        parameters.estimateCenter(),
        parameters.estimateTangent(),
        parameters.polinomDegree(),
        parameters.evenPowersOnly()
    );

    FunctionArgs *costFuntion = NULL;
    for (int costType = 0; costType < LineDistortionEstimatorCost::LINE_DISTORTION_ESTIMATOR_COST_LAST; costType++)
    {

        if (costType == LineDistortionEstimatorCost::JOINT_ANGLE_COST) {
            costFuntion = new AnglePointsFunction (straights, modelFactory);
        } else {
            costFuntion = new DistPointsFunction  (straights, modelFactory);
        }

        vector<double> modelParameters(costFuntion->inputs, 0);
        modelFactory.getModel(correction, &modelParameters[0]);

        vector<double> result(costFuntion->outputs);
        costFuntion->operator()(&modelParameters[0], &result[0]);

        double sqSum = 0;
        double maxValue = 0.0;
        for (unsigned i = 0; i < result.size(); i++) {
            if (fabs(result[i]) > maxValue) {
                maxValue = fabs(result[i]);
            }
            sqSum += result[i] * result[i];
        }

        if (updatePoints && costType == parameters.costAlgorithm())
        {
            int count = 0;
            for (unsigned i = 0; i < (unsigned)lineList->mPaths.size(); i++)
            {
                SelectableGeometryFeatures::VertexPath *path = lineList->mPaths[i];
                if (path->vertexes.size() < 3) {
                    continue;
                }
                if (parameters.costAlgorithm() == LineDistortionEstimatorCost::JOINT_ANGLE_COST)
                {
                    for (unsigned j = 1; j < path->vertexes.size() - 1; j++) {
                        path->vertexes[j]->weight = fabs(result[count]) / maxValue;
                        count++;
                    }
                } else {
                    for (unsigned j = 0; j < path->vertexes.size(); j++) {
                        path->vertexes[j]->weight = fabs(result[count]) / maxValue;
                        count++;
                    }
                }
            }
        }

        costs[costType] = sqrt(sqSum / result.size());
    }
}
