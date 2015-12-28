#include <algorithm>

#include "affine.h"

#include "cameraFixture.h"
#include "fixtureScene.h"

using namespace corecvs;

/**
 *  Camera  World   |  World   Camera
 *    X      -Y     |    X        Z
 *    Y      -Z     |    Y       -X
 *    Z       X     |    Z       -Y
 **/
FixtureScene::FixtureScene() :
     //worldFrameToCameraFrame(Affine3DQ::RotationZ(degToRad(90.0)) * Affine3DQ::RotationX(degToRad(90.0)))
    worldFrameToCameraFrame(Affine3DQ(Quaternion::FromMatrix(
        Matrix33( 0, -1,  0,
                  0,  0, -1,
                  1,  0,  0
        ))))
{
}

void FixtureScene::projectForward(SceneFeaturePoint::PointType mask, bool round)
{
    for (size_t pointId = 0; pointId < points.size(); pointId++)
    {
        SceneFeaturePoint *point = points[pointId];
        if ( (point->type & mask) == 0)
            continue;

        //cout << "Projecting point:" << point->name << " (" << point->position << ")"<< endl;

        for (size_t stationId = 0; stationId < stations.size(); stationId++)
        {
            CameraFixture &station = *stations[stationId];
            for (size_t camId = 0; camId < station.cameras.size(); camId++)
            {
                FixtureCamera *camera = station.cameras[camId];
                CameraModel worldCam = station.getWorldCamera((int)camId);


                Vector2dd projection = worldCam.project(point->position);
                if (!worldCam.isVisible(projection) || !worldCam.isInFront(point->position))
                    continue;

                if (round) {
                    projection.x() = fround(projection.x());
                    projection.y() = fround(projection.y());
                }

                SceneObservation observation;
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

FixtureCamera *FixtureScene::createCamera()
{
    FixtureCamera *camera = fabricateCamera();
    mOwnedObjects.push_back(camera);
    orphanCameras.push_back(camera);
    return camera;
}

CameraFixture *FixtureScene::createCameraFixture()
{
    CameraFixture *fixture = fabricateCameraFixture();
    mOwnedObjects.push_back(fixture);
    stations.push_back(fixture);
    return fixture;
}

SceneFeaturePoint *FixtureScene::createFeaturePoint()
{
    SceneFeaturePoint *point = fabricateFeaturePoint();
    mOwnedObjects.push_back(point);
    points.push_back(point);
    return point;
}

/* This method assumes the scene is well formed */
void FixtureScene::deleteCamera(CameraModel *camera)
{
    orphanCameras.erase( std::remove( orphanCameras.begin(), orphanCameras.end(), camera ), orphanCameras.end() );

    for(size_t i = 0; i < stations.size(); i++)
    {
        CameraFixture *station = stations[i];
        if (station == NULL) continue;
        station->cameras.erase( std::remove( station->cameras.begin(), station->cameras.end(), camera ), station->cameras.end() );
    }

    for(size_t i = 0; i < points.size(); i++)
    {
        SceneFeaturePoint *point = points[i];
        if (point == NULL) continue;

        auto it = point->observations.find(camera);
        if ( it != point->observations.end() ) {
            point->observations.erase(it);
        }
    }


    delete_safe(camera);

}

void FixtureScene::deleteCameraFixture(CameraFixture *fixture, bool recursive)
{
    if (recursive)
    {
        while (!fixture->cameras.empty()) {
            deleteCamera(fixture->cameras.back());
        }
    } else {
        orphanCameras.insert(orphanCameras.end(), fixture->cameras.begin(), fixture->cameras.end());
    }

    stations.erase( std::remove( stations.begin(), stations.end(), fixture ), stations.end() );
}

void FixtureScene::deleteFeaturePoint(SceneFeaturePoint *point)
{
    points.erase( std::remove( points.begin(), points.end(), point ), points.end() );
}

/**
 *  We are checking all parent links to be sure that we have a DAG.
 *  All double links will also be found
 *
 **/
bool FixtureScene::checkIntegrity()
{
    bool ok = true;

    for(size_t i = 0; i < orphanCameras.size(); i++)
    {
        CameraModel *cam = orphanCameras[i];
        if (cam == NULL) {
             ok = false; SYNC_PRINT(("Orphan Camera is NULL: scene:<%s> pos <%d>\n", this->nameId.c_str(), (int)i));
        }
        if (cam->ownerScene != this) {
             ok = false; SYNC_PRINT(("Orphan Camera form other scene: cam:<%s> scene:<%s>\n", cam->nameId.c_str(), this->nameId.c_str()));
        }
        if (cam->station != NULL) {
             ok = false; SYNC_PRINT(("Orphan Camera pretends to have station: cam:<%s>cam->station:<%s> scene:<%s>\n", cam->nameId.c_str(), cam->station->name.c_str(), this->nameId.c_str()));
        }
    }

    for(size_t i = 0; i < stations.size(); i++)
    {
        CameraFixture *station = stations[i];
        if (station == NULL) {
            ok = false; SYNC_PRINT(("Station is NULL: scene:<%s> pos <%d>\n", this->nameId.c_str(), (int)i));
        }
        if (station->ownerScene != this) {
            ok = false; SYNC_PRINT(("Station form other scene: station:<%s> scene:<%s>\n", station->name.c_str(), this->nameId.c_str()));
        }

        for(size_t j = 0; j < station->cameras.size(); j++)
        {
            CameraModel *cam = station->cameras[j];
            if (cam == NULL) {
                ok = false; SYNC_PRINT(("Station Camera is NULL: scene:<%s> station:<%s> pos <%d>\n", this->nameId.c_str(), station->name.c_str(), (int)j));
            }
            if (cam->ownerScene != this) {
                ok = false; SYNC_PRINT(("Station Camera form other scene: cam:<%s> station:<%s> scene:<%s>\n", cam->nameId.c_str(), station->name.c_str(), this->nameId.c_str()));
            }

            if (cam->station != station) {
                if (cam->station) {
                    ok = false; SYNC_PRINT(("Station Camera has NULL station: cam:<%s> station:<%s> scene:<%s>\n", cam->nameId.c_str(), station->name.c_str(), this->nameId.c_str()));
                } else {
                    ok = false;SYNC_PRINT(("Station Camera form other station: cam:<%s> station:<%s> cam->station:<%s> scene:<%s>\n", cam->nameId.c_str(), station->name.c_str(), cam->station->name.c_str(), this->nameId.c_str()));
                }
            }

        }

    }

    for(size_t i = 0; i < points.size(); i++)
    {
        CalibrationFeaturePoint *point = points[i];
        if (point == NULL) {
            ok = false; SYNC_PRINT(("Point is NULL: scene:<%s> pos <%d>\n", this->nameId.c_str(), (int)i));
        }
        if (point->ownerScene != this) {
            ok = false; SYNC_PRINT(("Point form other scene: point:<%s> scene:<%s>\n", point->name.c_str(), this->nameId.c_str()));
        }

        for (auto it = point->observations.begin(); it != point->observations.end(); ++it)
        {
            CameraModel *cam = it->first;
            const CalibrationObservation &observ = it->second;
            if (observ.camera == NULL)
            {
                ok = false; SYNC_PRINT(("observation has null camera"));
            }
            if (cam == NULL)
            {
                ok = false; SYNC_PRINT(("there is a null entry in observation"));
            }
            if (observ.camera != cam) {
                ok = false; SYNC_PRINT(("Point observation list malformed"));
            }

            if (observ.featurePoint != point) {
                ok = false; SYNC_PRINT(("Point observation has wrong point pointer"));
            }
        }
    }


    return ok;
}




void FixtureScene::positionCameraInFixture(CameraFixture *fixture, CameraModel *camera, const Affine3DQ &location)
{


//    cout << "CalibrationScene::positionCameraInStation()" << std::endl;
//    cout << "  World Transform():" << std::endl;
//    worldFrameToCameraFrame.prettyPrint1();
//    cout << "  Input:" << std::endl;
//    location.prettyPrint1();
    camera->extrinsics = CameraLocationData(location * worldFrameToCameraFrame.inverted());
//    cout << "  In camera form";
//    camera->extrinsics.prettyPrint1();
}

void FixtureScene::addCameraToStation(CameraModel *cam, corecvs::CameraFixture *station)
{

    auto it = std::find(orphanCameras.begin(), orphanCameras.end(), cam);
    if (it != orphanCameras.end()) {
        orphanCameras.erase(it);
    }
    cam->station = station;
    station->cameras.push_back(cam);

}

FixtureScene::~FixtureScene()
{
    for(size_t i = 0; i < mOwnedObjects.size(); i++)
    {
        delete_safe(mOwnedObjects[i]);
    }
}

FixtureCamera *corecvs::FixtureScene::fabricateCamera()
{
   return new CameraModel(this);
}

CameraFixture *FixtureScene::fabricateCameraFixture()
{
    return new Photostation(this);
}

CalibrationFeaturePoint *FixtureScene::fabricateFeaturePoint()
{
    return new CalibrationFeaturePoint(this);
}
