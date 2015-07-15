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
                1.0);


    return RadialCorrection(lenCorrectParams);
 }


LMLinesDistortionSolver::LMLinesDistortionSolver()
{

}

RadialCorrection LMLinesDistortionSolver::solve()
{
    vector<vector<Vector2dd> > straights = lineList->getLines();
    L_INFO_P("Starting distortion calibration on %d lines", straights.size());
/*
    RadialCorrection correction(LensDistortionModelParameters(
       vector<double>(mUi->degreeSpinBox->value()),
       0.0, 0.0,
       1.0,
       center.l2Metric(),
       center
    ));
 */

    RadialCorrection correction(LensDistortionModelParameters(
       initialCenter.x(),
       initialCenter.y(),
       0.0 ,0.0,
       vector<double>(parameters.polinomDegree()),
       1.0,
       1.0
    ));


    ModelToRadialCorrection modelFactory(
        correction,
        parameters.estimateCenter(),
        parameters.estimateTangent(),
        parameters.polinomDegree()
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


  //  L_INFO_P("Inverting the distortion buffer");

  /* mDistortionCorrectTransform = QSharedPointer<DisplacementBuffer>(DisplacementBuffer::CacheInverse(&mLinesRadialCoorection,
            mBufferInput->h, mBufferInput->w,
            0.0,0.0,
            (double)mBufferInput->w, (double)mBufferInput->h,
            0.5, false)
   );

    mUi->isInverseCheckBox->setChecked(true);

    mUi->lensCorrectionWidget->setParameters(mLinesRadialCoorection.mParams);*/
    L_INFO_P("Ending distortion calibration");
//    updateScore();
    return modelFactory.getRadial(&straightParams[0]);

}
