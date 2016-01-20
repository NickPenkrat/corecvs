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

        for (size_t stationId = 0; stationId < fixtures.size(); stationId++)
        {
            CameraFixture &station = *fixtures[stationId];
            for (size_t camId = 0; camId < station.cameras.size(); camId++)
            {
                FixtureCamera *camera = station.cameras[camId];
                CameraModel worldCam = station.getWorldCamera(camera);


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
                point->observations__[SceneFeaturePoint::WPP(fixtures[stationId], camera)] = observation;
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
    fixtures.push_back(fixture);
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
void FixtureScene::deleteCamera(FixtureCamera *camera)
{
    orphanCameras.erase( std::remove( orphanCameras.begin(), orphanCameras.end(), camera ), orphanCameras.end() );

    for(size_t i = 0; i < fixtures.size(); i++)
    {
        CameraFixture *station = fixtures[i];
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

        SceneFeaturePoint::WPP wpp(SceneFeaturePoint::WPP::UWILDCARD, camera);
        for (auto it = point->observations__.begin(); it != point->observations__.end(); it = it->first == wpp ? it = point->observations__.erase(it) : it++);
    }


    delete_safe(camera);

}

void FixtureScene::deleteCameraFixture(CameraFixture *fixture, bool recursive)
{
    for(size_t i = 0; i < points.size(); i++)
    {
        SceneFeaturePoint *point = points[i];
        if (point == NULL) continue;

        SceneFeaturePoint::WPP wpp(fixture, SceneFeaturePoint::WPP::VWILDCARD);
        for (auto it = point->observations__.begin(); it != point->observations__.end(); it = it->first == wpp ? it = point->observations__.erase(it) : it++);
    }
    if (recursive)
    {
        while (!fixture->cameras.empty()) {
            deleteCamera(fixture->cameras.back());
        }
    } else {
        orphanCameras.insert(orphanCameras.end(), fixture->cameras.begin(), fixture->cameras.end());
    }

    fixtures.erase( std::remove( fixtures.begin(), fixtures.end(), fixture ), fixtures.end() );
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
        FixtureCamera *cam = orphanCameras[i];
        if (cam == NULL) {
             ok = false; SYNC_PRINT(("Orphan Camera is NULL: scene:<%s> pos <%d>\n", this->nameId.c_str(), (int)i));
        }
        if (cam->ownerScene != this) {
             ok = false; SYNC_PRINT(("Orphan Camera form other scene: cam:<%s> scene:<%s>\n", cam->nameId.c_str(), this->nameId.c_str()));
        }
        if (cam->cameraFixture != NULL) {
             ok = false; SYNC_PRINT(("Orphan Camera pretends to have station: cam:<%s>cam->station:<%s> scene:<%s>\n", cam->nameId.c_str(), cam->cameraFixture->name.c_str(), this->nameId.c_str()));
        }
    }

    for(size_t i = 0; i < fixtures.size(); i++)
    {
        CameraFixture *fixture = fixtures[i];
        if (fixture == NULL) {
            ok = false; SYNC_PRINT(("Station is NULL: scene:<%s> pos <%d>\n", this->nameId.c_str(), (int)i));
        }
        if (fixture->ownerScene != this) {
            ok = false; SYNC_PRINT(("Station form other scene: station:<%s> scene:<%s>\n", fixture->name.c_str(), this->nameId.c_str()));
        }

        for(size_t j = 0; j < fixture->cameras.size(); j++)
        {
            FixtureCamera *cam = fixture->cameras[j];
            if (cam == NULL) {
                ok = false; SYNC_PRINT(("Station Camera is NULL: scene:<%s> station:<%s> pos <%d>\n", this->nameId.c_str(), fixture->name.c_str(), (int)j));
            }
            if (cam->ownerScene != this) {
                ok = false; SYNC_PRINT(("Station Camera form other scene: cam:<%s> station:<%s> scene:<%s>\n", cam->nameId.c_str(), fixture->name.c_str(), this->nameId.c_str()));
            }
#if 0
            if (cam->cameraFixture != fixture) {
                if (cam->cameraFixture) {
                    ok = false; SYNC_PRINT(("Station Camera has NULL station: cam:<%s> station:<%s> scene:<%s>\n", cam->nameId.c_str(), fixture->name.c_str(), this->nameId.c_str()));
                } else {
                    ok = false;SYNC_PRINT(("Station Camera form other station: cam:<%s> station:<%s> cam->station:<%s> scene:<%s>\n", cam->nameId.c_str(), fixture->name.c_str(), cam->cameraFixture->name.c_str(), this->nameId.c_str()));
                }
            }
#endif

        }

    }

    for(size_t i = 0; i < points.size(); i++)
    {
        SceneFeaturePoint *point = points[i];
        if (point == NULL) {
            ok = false; SYNC_PRINT(("Point is NULL: scene:<%s> pos <%d>\n", this->nameId.c_str(), (int)i));
        }
        if (point->ownerScene != this) {
            ok = false; SYNC_PRINT(("Point form other scene: point:<%s> scene:<%s>\n", point->name.c_str(), this->nameId.c_str()));
        }

        for (auto it = point->observations.begin(); it != point->observations.end(); ++it)
        {
            CameraModel *cam = it->first;
            const SceneObservation &observ = it->second;
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
        for (auto& it: point->observations__)
        {
            FixtureCamera* fixtureCamera = it.first.v;
            CameraFixture* cameraFixture = it.first.u;
            const SceneObservation &observ = it.second;

            if (observ.camera == NULL)
            {
                ok = false; SYNC_PRINT(("observation__ has null camera"));
            }
            if (observ.cameraFixture == NULL)
            {
                ok = false; SYNC_PRINT(("observation__ has null camera fixture"));
            }
            if (fixtureCamera == SceneFeaturePoint::WPP::VWILDCARD)
            {
                ok = false; SYNC_PRINT(("there is a wild-card entry in observation__"));
            }
            if (cameraFixture == SceneFeaturePoint::WPP::UWILDCARD)
            {
                ok = false; SYNC_PRINT(("there is a wild-card entry in observation__"));
            }
            if (observ.camera != fixtureCamera)
            {
                ok = false; SYNC_PRINT(("Point observation__ list malformed"));
            }
            if (observ.cameraFixture != cameraFixture)
            {
                ok = false; SYNC_PRINT(("Point observation__ list malformed"));
            }
            if (observ.featurePoint != point)
            {
                ok = false; SYNC_PRINT(("Point observation__ has wrong point pointer"));
            }

        }
    }


    return ok;
}




void FixtureScene::positionCameraInFixture(CameraFixture *fixture, FixtureCamera *camera, const Affine3DQ &location)
{


//    cout << "FixtureScene::positionCameraInStation()" << std::endl;
//    cout << "  World Transform():" << std::endl;
//    worldFrameToCameraFrame.prettyPrint1();
//    cout << "  Input:" << std::endl;
//    location.prettyPrint1();
    camera->extrinsics = CameraLocationData(location * worldFrameToCameraFrame.inverted());
//    cout << "  In camera form";
//    camera->extrinsics.prettyPrint1();
}

void FixtureScene::addCameraToFixture(FixtureCamera *cam, CameraFixture *fixture)
{

    auto it = std::find(orphanCameras.begin(), orphanCameras.end(), cam);
    if (it != orphanCameras.end()) {
        orphanCameras.erase(it);
    }
    cam->cameraFixture = fixture;
    fixture->cameras.push_back(cam);

}

void FixtureScene::dumpInfo(ostream &out)
{
    out << "FixtureScene:" << endl;
    out << "Owned objects: " <<  mOwnedObjects.size() << endl;

    out << "Orphan Cameras: " <<  orphanCameras.size() << endl;
    out << "Fixtures: " <<  fixtures.size() << endl;
    for(size_t i = 0; i < fixtures.size(); i++)
    {
        CameraFixture *fixture = fixtures[i];
        out << "  " << "Fixture <" << fixture->name << "> " << fixture->cameras.size() << endl;
        for(size_t j = 0; j < fixture->cameras.size(); j++)
        {
            FixtureCamera *cam = fixture->cameras[j];
            out << "     " << "Camera <" << cam->nameId << "> "  << endl;
        }
    }

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
   return new FixtureCamera(this);
}

CameraFixture *FixtureScene::fabricateCameraFixture()
{
    return new CameraFixture(this);
}

SceneFeaturePoint *FixtureScene::fabricateFeaturePoint()
{
    return new SceneFeaturePoint(this);
}
