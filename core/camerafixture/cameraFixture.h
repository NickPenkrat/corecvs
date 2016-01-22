#pragma once
/**
 * \file cameraFixture.h
 * \ingroup cppcorefiles
 *
 **/

#include <type_traits>
#include <cstring>

#include "pointObservation.h"

#include "typesafeBitmaskEnums.h"
#include "calibrationLocation.h"  // LocationData
#include "fixtureCamera.h"
//#include "fixtureScene.h"

namespace corecvs {

/**
 *   See FixtureScene for more data on ownership of the objectes in structure
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

    FixtureCamera getWorldCamera(FixtureCamera *camPtr) const
    {
        FixtureCamera toReturn = *camPtr;
        toReturn.extrinsics.transform(location);
        return toReturn;
    }

    FixtureCamera getWorldCamera(int cam) const
    {
        return getWorldCamera(cameras[cam]);
    }

    FixtureCamera getRawCamera(int cam) const
    {
     /*   auto c = cameras[cam];
        c.extrinsics.orientation = c.extrinsics.orientation ^ location.orientation;
        c.extrinsics.position = (location.orientation.conjugated() * cameras[cam].extrinsics.position) + location.position;
        return c;*/
        return getWorldCamera(cam);
    }

    int getCameraId(FixtureCamera* ptr) const
    {
        for (auto& i: cameras)
            if (i == ptr)
                return &i - &cameras[0];
        return -1;
    }

    void setCameraCount(int count);

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

} // namespace corecvs
