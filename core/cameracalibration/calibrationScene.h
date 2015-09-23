#ifndef CALIBRATIONSCENE_H
#define CALIBRATIONSCENE_H

#include "calibrationCamera.h"
#include "calibrationPhotostation.h"

/* Heap of Calibration related stuff */

class CalibrationScene
{
public:
    CalibrationScene();

    vector<ScenePart *> mOwnedObjects;


};

#endif // CALIBRATIONSCENE_H
