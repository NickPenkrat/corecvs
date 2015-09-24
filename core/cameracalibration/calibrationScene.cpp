#include "calibrationScene.h"

CalibrationScene::CalibrationScene()
{

}

void CalibrationScene::projectForward(CalibrationFeaturePoint::PointType mask)
{
    for (size_t pointId = 0; pointId < points.size(); pointId++)
    {
        CalibrationFeaturePoint *point = &(points[pointId]);
        if ( (point->type & mask) == 0)
            continue;

        for (size_t stationId = 0; stationId < stations.size(); stationId++)
        {
            Photostation &station = stations[stationId];
            for (size_t camId = 0; camId < station.cameras.size(); camId++)
            {
                CameraModel *camera = &(station.cameras[camId]);
                Vector2dd projection = camera->project(point->position);
                if (!camera->isVisible(projection))
                    continue;

                CalibrationObservation observation;
                observation.accuracy = Vector2dd(0.0);
                observation.camera = camera;
                observation.featurePoint = point;
                observation.isKnown = true;
                observation.observation = projection;
                point->observations[camera] = observation;
            }
        }
    }
}

