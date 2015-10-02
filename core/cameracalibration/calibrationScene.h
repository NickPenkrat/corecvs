#pragma once

#include "calibrationCamera.h" // ScenePart
#include "calibrationPhotostation.h"
#include "calibrationFeaturePoint.h"

/* Heap of Calibration related stuff */

class CalibrationScene
{
public:
    CalibrationScene();

    /* This is for future, when all the heap/memory will be completed */
    vector<ScenePart *>             mOwnedObjects;

    vector<Photostation>            stations;
    vector<CalibrationFeaturePoint> points;

    /** **/
    void projectForward(CalibrationFeaturePoint::PointType mask);

    /* Manipulation with structures */
    void addCameraToStation(CameraModel *cam, Photostation *station)
    {
        station->cameras.push_back(*cam);
        cam->station = station;
    }

};
