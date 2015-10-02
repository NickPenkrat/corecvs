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

    /**
     *  Creates and fills the observations with points. It optionally simulates camera by rounding the projection to nearest pixel
     *
     **/
    void projectForward(CalibrationFeaturePoint::PointType mask, bool round = false);

    /* Manipulation with structures */
    void addCameraToStation(CameraModel * cam, Photostation *station)
    {
        station->cameras.push_back(*cam);
        cam->station = station;
    }

    size_t totalObservations()
    {
        size_t toReturn = 0;
        for (size_t pointId = 0; pointId < points.size(); pointId++)
        {
            CalibrationFeaturePoint *point = &(points[pointId]);
            toReturn += point->observations.size();
        }
        return toReturn;
    }
};

#endif // CALIBRATIONSCENE_H
