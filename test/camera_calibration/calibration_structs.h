#ifndef CALIBRATION_STRUCTS
#define CALIBRATION_STRUCTS

#include <type_traits>

#include "quaternion.h"
#include "matrix33.h"
#include "vector3d.h"
#include "vector2d.h"
#include "typesafeBitmaskEnums.h"

enum class CameraConstraints 
{
    NONE           =  0,
    ZERO_SKEW      =  1, // This forces skew to zero, but not locks it to zero during non-linear optimization
    LOCK_SKEW      =  2, // This one locks skew, but not forces it to zero
    EQUAL_FOCAL    =  4, // Makes fx = fy in non-linear phase
    LOCK_FOCAL     =  8, // Locks fx and fy
    LOCK_PRINCIPAL = 16, // Locks cx and cy

};

template<>
struct is_bitmask<CameraConstraints> : std::true_type {};

struct LocationData
{
    LocationData(corecvs::Vector3dd position = corecvs::Vector3dd(0.0, 0.0, 1.0), corecvs::Quaternion orientation = corecvs::Quaternion(0.0, 0.0, 0.0, 1.0)) : position(position), orientation(orientation)
    {
    }

	corecvs::Vector3dd position;
	corecvs::Quaternion orientation;
};

// TODO: We already have intrinsics class somewhere, don't we?
struct CameraIntrinsics_
{
    CameraIntrinsics_(double fx = 1.0, double fy = 1.0, double cx = 0.0, double cy = 0.0, double skew = 0.0) : fx(fx), fy(fy), cx(cx), cy(cy), skew(skew)
    {
    }
 
    // The idea is that if we merge distorsion calibration WITH extrinsics/intrinsics
    // calibration, then this method will project point using forward distorsion
    // map
    corecvs::Vector2dd project(const corecvs::Vector3dd &pt)
    {
        double normalizer = pt[2];
        return corecvs::Vector2dd(fx * pt[0] + skew * pt[1] + cx * pt[2], fy * pt[1] + cy * pt[2]) / normalizer;
    }
	
	explicit operator corecvs::Matrix33() const
    {
        return corecvs::Matrix33(fx, skew, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    }
	
	double fx, fy, cx, cy, skew;
};

struct Camera
{
    corecvs::Vector2dd project(const corecvs::Vector3dd &pt)
    {
        return intrinsics.project(extrinsics.orientation * (pt - extrinsics.position));
    }

    CameraIntrinsics_ intrinsics;
	LocationData extrinsics;
};

struct Photostation
{
    corecvs::Vector2dd project(const corecvs::Vector3dd &pt, int cam)
    {
        return cameras[cam].project(location.orientation * (pt - location.position));
    }

	std::vector<Camera> cameras;
	LocationData location;
};

typedef std::vector<std::pair<corecvs::Vector2dd, corecvs::Vector3dd>> PatternPoints3d;
typedef std::vector<PatternPoints3d> MultiCameraPatternPoints;


#endif