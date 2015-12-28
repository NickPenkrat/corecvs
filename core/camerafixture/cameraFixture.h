#pragma once
/**
 * \file cameraFixture.h
 * \ingroup cppcorefiles
 *
 **/

#include <type_traits>
#include <cstring>

#include "pointObservation.h"
#include "selectableGeometryFeatures.h"

#include "typesafeBitmaskEnums.h"
#include "calibrationLocation.h"  // LocationData
#include "fixtureCamera.h"
#include "fixtureScene.h"

namespace corecvs {

/**
 *   See CalibrationScene for more data on ownership of the objectes in structure
 **/
class CameraFixture : public FixtureScenePart
{
public:
    std::vector<FixtureCamera *> cameras;
    Affine3DQ                    location;
    std::string                  name;

    CameraFixture(FixtureScene * owner = NULL) :
        FixtureScenePart(owner)
    {}

    CameraFixture(
        const std::vector<FixtureCamera *> & _cameras,
        const Affine3DQ &_location = Affine3DQ())
      : cameras(_cameras)
      , location(_location)
    {}

    /** This is a legacy compatibilty block **/

    CameraFixture(
        const std::vector<FixtureCamera *> & _cameras,
        const CameraLocationData &_location)
      : cameras(_cameras)
      , location(_location.toMockAffine3D())
    {}

    CameraLocationData getLocation() const
    {
        return CameraLocationData(location);
    }

    void setLocation(const CameraLocationData &_location)
    {
        location = _location.toMockAffine3D();
    }

    /* New style setter */
    void setLocation(const Affine3DQ &_location)
    {
        location = _location;
    }

    CameraModel getWorldCamera(CameraModel *camPtr) const
    {
        CameraModel toReturn = *camPtr;
        toReturn.extrinsics.transform(location);
        return toReturn;
    }

    CameraModel getWorldCamera(int cam) const
    {
        return getWorldCamera(cameras[cam]);
    }

    CameraModel getRawCamera(int cam) const
    {
     /*   auto c = cameras[cam];
        c.extrinsics.orientation = c.extrinsics.orientation ^ location.orientation;
        c.extrinsics.position = (location.orientation.conjugated() * cameras[cam].extrinsics.position) + location.position;
        return c;*/
        return getWorldCamera(cam);
    }

    void setCameraCount(int count) {
        while  (cameras.size() > (size_t)count) {
            FixtureCamera *model = cameras.back();
            cameras.pop_back(); /* delete camera will generally do it, but only in owner scene.*/
            model->ownerScene->deleteCamera(model);
        }

        while  (cameras.size() < (size_t)count) {
            FixtureCamera *model  = ownerScene->createCamera();
            ownerScene->addCameraToStation(model, this);
        }
    }
    
    Matrix44 getMMatrix(int cam) const
    {
        return getRawCamera(cam).getCameraMatrix();
    }

    Vector2dd project(const Vector3dd &pt, int cam) const
    {
        return cameras[cam]->project(location.inverted().apply(pt));
    }

    bool isVisible(const Vector3dd &pt, int cam) const
    {
        return cameras[cam]->isVisible(location.inverted().apply(pt));
    }

    bool isVisible(const Vector3dd &pt) const
    {
        for (int i = 0; i < (int)cameras.size(); ++i) {
            if (isVisible(pt, i))
                return true;
        }
        return false;
    }


    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        /* So far compatibilty is on */
        int camsize = cameras.size();
        visitor.visit(camsize, 0, "cameras.size");

        setCameraCount(camsize);

        for (size_t i = 0; i < (size_t)camsize; i++)
        {
            visitor.visit(*cameras[i], "cameras");
        }

        CameraLocationData loc = getLocation();
        visitor.visit(loc, CameraLocationData(), "location");
        setLocation(loc);
    }
};

//typedef std::vector<PointObservation> PatternPoints3d;
typedef std::vector<ObservationList>  MultiCameraPatternPoints;

} // namespace corecvs

