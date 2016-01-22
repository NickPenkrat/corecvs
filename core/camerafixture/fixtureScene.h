#ifndef FIXTURE_SCENE_H_
#define FIXTURE_SCENE_H_

#include "fixtureCamera.h"
#include "cameraFixture.h"
#include "sceneFeaturePoint.h"

namespace corecvs {

class CameraFixture;


/* Heap of Calibration related stuff */

class FixtureScene
{
public:
    FixtureScene();

    /**
     * This field encodes the "silent transform that happens when natural world corrdinate system changes to
     * image related. This covers but is not restritced to the transition between:
     *
     *   Z axis pointing to the sky and Z axis pointing to camera optical axis
     *
     * So far you can't change right handed system to left-handed. This cavity needs to be addressed later.
     *
     *  Camera  World   |  World   Camera
     *    Z       X     |    X        Z
     *    Y      -Z     |    Y       -X
     *    X      -Y     |    Z       -Y
     *
     *  This transform only happens when you use ::positionCameraInStation() method. Thoough we encourage you to do so.
     *
     **/
    Affine3DQ worldFrameToCameraFrame;

    std::string nameId;

    /* This is for future, when all the heap/memory will be completed */
    vector<FixtureScenePart *>    mOwnedObjects;

    vector<CameraFixture *>       fixtures;
    vector<FixtureCamera *>       orphanCameras;
    vector<SceneFeaturePoint *>   points;

    /**
     *  Creates and fills the observations with points. It optionally simulates camera by rounding the projection to nearest pixel
     *
     **/
    void projectForward(SceneFeaturePoint::PointType mask, bool round = false);

protected:
    virtual FixtureCamera      *fabricateCamera();
    virtual CameraFixture      *fabricateCameraFixture();
    virtual SceneFeaturePoint  *fabricateFeaturePoint();


public:

    /**
     * Manipulation with structures
     **/
    virtual FixtureCamera      *createCamera();
    virtual CameraFixture      *createCameraFixture();
    virtual SceneFeaturePoint  *createFeaturePoint();

    /* These methods  compleatly purge camera from scene */
    virtual void deleteCamera        (FixtureCamera *camera);
    virtual void deleteCameraFixture (CameraFixture *fixture, bool recursive = true);
    virtual void deleteFeaturePoint  (SceneFeaturePoint *camera);


    /**
     *    Helper method to check data structure integrity
     *
     *    The restritions in this method are strongly recomended but not enforced
     *
     **/
    virtual bool checkIntegrity();

    virtual void positionCameraInFixture(CameraFixture *station, FixtureCamera *camera, const Affine3DQ &location);
    virtual void addCameraToFixture     (FixtureCamera *cam, CameraFixture *fixture);

    /* Some debugger helpers */
    virtual void dumpInfo(ostream &out);


    size_t totalObservations()
    {
        size_t toReturn = 0;
        for (size_t pointId = 0; pointId < points.size(); pointId++)
        {
            const SceneFeaturePoint *point = points[pointId];
            toReturn += point->observations.size();
        }
        return toReturn;
    }

    /**
     *
     **/
    void setFixtureCount(int count);
    void setOrphanCameraCount(int count);
    void setFeaturePointCount(int count);


    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        /* So far compatibilty is on */
        /* Orphan cameras */
        int ocamSize = orphanCameras.size();
        visitor.visit(ocamSize, 0, "orphancameras.size");

        setOrphanCameraCount(ocamSize);

        for (size_t i = 0; i < (size_t)ocamSize; i++)
        {
            char buffer[100];
            snprintf2buf(buffer, "orphancameras[%d]", i);
            visitor.visit(*orphanCameras[i], buffer);
        }

        /* Fixtures*/

        int stationSize = orphanCameras.size();
        visitor.visit(stationSize, 0, "stations.size");

        setOrphanCameraCount(stationSize);

        for (size_t i = 0; i < (size_t)stationSize; i++)
        {
            char buffer[100];
            snprintf2buf(buffer, "stations[%d]", i);
            visitor.visit(*fixtures[i], buffer);
        }

        /* Points */

    }

    virtual ~FixtureScene();
};

} // namespace corecvs


#endif // FIXTURE_SCENE_H_
