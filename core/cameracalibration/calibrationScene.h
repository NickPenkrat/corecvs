#ifndef CALIBRATIONSCENE_H
#define CALIBRATIONSCENE_H

#include "calibrationCamera.h"
#include "calibrationPhotostation.h"
#include "calibrationFeaturePoint.h"

/* Heap of Calibration related stuff */

class CalibrationScene
{
public:
    CalibrationScene();

    /* This is for future, when all the heap/memory will be completed */
    vector<ScenePart *> mOwnedObjects;


    vector<Photostation> stations;
    vector<CalibrationFeaturePoint> points;

    void projectForward(CalibrationFeaturePoint::PointType mask);


};

#endif // CALIBRATIONSCENE_H
