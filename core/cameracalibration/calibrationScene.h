#pragma once

#include "calibrationCamera.h" // ScenePart
#include "calibrationPhotostation.h"
#include "calibrationFeaturePoint.h"

namespace corecvs {

/* Heap of Calibration related stuff */

class CalibrationScene
{
public:
    CalibrationScene();

    /* This is for future, when all the heap/memory will be completed */
    vector<ScenePart *>             mOwnedObjects;

    vector<Photostation *>            stations;
    vector<CameraModel *>             orphanCameras;
    vector<CalibrationFeaturePoint *> points;

    /**
     *  Creates and fills the observations with points. It optionally simulates camera by rounding the projection to nearest pixel
     *
     **/
    void projectForward(CalibrationFeaturePoint::PointType mask, bool round = false);

protected:
    virtual CameraModel              *fabricateCamera();
    virtual Photostation             *fabricatePhotostation();
    virtual CalibrationFeaturePoint  *fabricateFeaturePoint();


public:

    /**
     * Manipulation with structures
     **/
    virtual CameraModel              *createCamera();
    virtual Photostation             *createPhotostation();
    virtual CalibrationFeaturePoint  *createFeaturePoint();


    virtual void addCameraToStation(CameraModel *cam, Photostation *station);



    size_t totalObservations()
    {
        size_t toReturn = 0;
        for (size_t pointId = 0; pointId < points.size(); pointId++)
        {
            const CalibrationFeaturePoint *point = points[pointId];
            toReturn += point->observations.size();
        }
        return toReturn;
    }

    virtual ~CalibrationScene();
};

} // namespace corecvs
