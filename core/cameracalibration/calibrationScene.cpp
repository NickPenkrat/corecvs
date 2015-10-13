#include "calibrationScene.h"

CalibrationScene::CalibrationScene()
{
}

void CalibrationScene::projectForward(CalibrationFeaturePoint::PointType mask, bool round)
{
    for (size_t pointId = 0; pointId < points.size(); pointId++)
    {
        CalibrationFeaturePoint *point = &(points[pointId]);
        if ( (point->type & mask) == 0)
            continue;

        //cout << "Projecting point:" << point->name << " (" << point->position << ")"<< endl;

        for (size_t stationId = 0; stationId < stations.size(); stationId++)
        {
            Photostation &station = stations[stationId];
            for (size_t camId = 0; camId < station.cameras.size(); camId++)
            {
                CameraModel *camera = &(station.cameras[camId]);
                CameraModel worldCam = station.getWorldCamera((int)camId);


                Vector2dd projection = worldCam.project(point->position);
                if (!worldCam.isVisible(projection) || !worldCam.isInFront(point->position))
                    continue;

                if (round) {
                    projection.x() = fround(projection.x());
                    projection.y() = fround(projection.y());
                }

                CalibrationObservation observation;
                observation.accuracy     = Vector2dd(0.0);
                observation.camera       = camera;
                observation.featurePoint = point;
                observation.isKnown      = true;
                observation.observation  = projection;

                Vector3dd direct = worldCam.dirToPoint(point->position).normalised();
                Vector3dd indirect = worldCam.intrinsics.reverse(projection).normalised();

                if (!round) {
                    observation.observDir = direct;
                } else {
                    observation.observDir = indirect;
                }

                /*if (direct.notTooFar(indirect, 1e-7))
                {
                    SYNC_PRINT(("Ok\n"));
                } else {
                    cout << direct << " - " << indirect << "  ";
                    SYNC_PRINT(("Fail\n"));
                }*/

                point->observations[camera] = observation;

                //cout << "Camera:" << camera->fileName << " = " << projection << endl;
            }
        }
    }
}