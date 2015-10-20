#pragma once

/**
 * \file calibrationPhotostation.h
 * \ingroup cppcorefiles
 *
 **/

#include <type_traits>
#include <cstring>

#include "typesafeBitmaskEnums.h"
#include "calibrationCamera.h"
#include "calibrationLocation.h"  // LocationData

namespace corecvs {

enum class CameraConstraints
{
    NONE           =  0x00,
    ZERO_SKEW      =  0x01, // This forces skew to zero, but not locks it to zero during non-linear optimization
    LOCK_SKEW      =  0x02, // This one locks skew, but not forces it to zero
    EQUAL_FOCAL    =  0x04, // Makes fx = fy in non-linear phase
    LOCK_FOCAL     =  0x08, // Locks fx and fy
    LOCK_PRINCIPAL =  0x10, // Locks cx and cy
    UNLOCK_YSCALE  =  0x20  // Unlock Y scale of pattern. This is dangerous if you are not sure what are you doing
};

} // namespace corecvs

template<>
struct is_bitmask<CameraConstraints> : std::true_type {};

namespace corecvs {


/**
 *   See CalibrationScene for more data on ownership of the objectes in structure
 **/
class Photostation : public ScenePart
{
public:
    std::vector<CameraModel> cameras;
    Affine3DQ                location;
    std::string              name;

    Photostation(CalibrationScene * owner = NULL) :
        ScenePart(owner)
    {}

    Photostation(
        const std::vector<CameraModel> & _cameras,
        const Affine3DQ &_location = Affine3DQ())
      : cameras(_cameras)
      , location(_location)
    {}

    /** This is a legacy compatibilty block **/

    Photostation(
        const std::vector<CameraModel> & _cameras,
        const CameraLocationData &_location)
      : cameras(_cameras)
      , location(_location.toAffine3D())
    {}

    CameraLocationData getLocation() const
    {
        return CameraLocationData(location);
    }

    void setLocation(const CameraLocationData &_location)
    {
        location = _location.toAffine3D();
    }

    /* New style setter */
    void setLocation(const Affine3DQ &_location)
    {
        location = _location;
    }


    CameraModel getWorldCamera(int cam) const
    {
        CameraModel toReturn = cameras[cam];
        toReturn.extrinsics.transform(location);
        return toReturn;
    }

    CameraModel getRawCamera(int cam) const
    {
     /*   auto c = cameras[cam];
        c.extrinsics.orientation = c.extrinsics.orientation ^ location.orientation;
        c.extrinsics.position = (location.orientation.conjugated() * cameras[cam].extrinsics.position) + location.position;
        return c;*/
        return getWorldCamera(cam);
    }
    
    Matrix44 getKMatrix(int cam) const
    {
        return getRawCamera(cam).getCameraMatrix();
    }

    Vector2dd project(const Vector3dd &pt, int cam) const
    {
        return cameras[cam].project(location.apply(pt));
    }

    bool isVisible(const Vector3dd &pt, int cam) const
    {
        return cameras[cam].isVisible(location.apply(pt));
    }


    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        /*
          visitor.visit(cameras, "cameras");
          visitor.visit(location, CameraLocationData(), "location");
        */

        /* So far comptibilty is on */
        visitor.visit(cameras, "cameras");

        CameraLocationData loc = getLocation();
        visitor.visit(loc, CameraLocationData(), "location");
        setLocation(loc);
    }
};

typedef std::vector<std::pair<Vector2dd, Vector3dd>> PatternPoints3d;
typedef std::vector<PatternPoints3d>                 MultiCameraPatternPoints;

} // namespace corecvs
