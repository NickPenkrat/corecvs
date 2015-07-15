#ifndef LMDISTORTIONSOLVER_H
#define LMDISTORTIONSOLVER_H

#include "radialCorrection.h"
#include "selectableGeometryFeatures.h"
#include "lineDistortionEstimatorParameters.h"

class LMDistortionSolver
{
public:
    LMDistortionSolver();

    ObservationList *list;
    //LineDistortionEstimatorParameters parameters;
    Vector2dd initialCenter;

    RadialCorrection solve();
};


class LMLinesDistortionSolver
{
public:
    LMLinesDistortionSolver();

    SelectableGeometryFeatures *lineList;
    LineDistortionEstimatorParameters parameters;
    Vector2dd initialCenter;

    RadialCorrection solve();
};




#endif // LMDISTORTIONSOLVER_H
