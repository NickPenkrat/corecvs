#ifndef CALIBRATIONCAMERA_H
#define CALIBRATIONCAMERA_H

#include <tbb/tbb_machine.h>
#include <unordered_map>

#include "vector2d.h"
#include "vector3d.h"
#include "quaternion.h"
#include "matrix44.h"
#include "line.h"
#include "convexPolyhedron.h"
#include "lensDistortionModelParameters.h"


namespace corecvs {


/**
 * This class is so far just a common base for all objects in scene heap.
 * Should bring this to other file
 **/
class ScenePart {
public:


    /* No particular reson for this, except to encourage leak checks */
    static int OBJECT_COUNT;

    /* We could have copy constructors and stuff... but so far this is enough */
    ScenePart() {
        OBJECT_COUNT++;

    }

    virtual ~ScenePart() {
        OBJECT_COUNT++;
    }
};


typedef std::unordered_map<std::string, void *> MetaContainer;



/**
 *   Contrary to what Affine3D does this class holds reference frame transformation in camera related terms
 *
 *
 *   Information about camera.
 *   Model is as follows
 *    1. The input world point is X (position)
 *    2. It is then transformed to homogeneous coordinates X'
 *    3. Then Camera Extrinsic parameters matrix is applied
 *       1. Point is moved so that camera becomes at 0
 *           X' = X - Position
 *       1. World is rotated (orientation) to meet the camera position
 *           E = Rotation * X'
 *    4. Then we apply Intrinsic camera transfromation
 *
 *    \f[
 *
 *   \pmatrix { u \cr v \cr t } =
 *
 *   \left( \begin{array}{ccc|c}
 *        \multicolumn{3}{c|}{\multirow{3}{*}{$R_{3 \times 3}$}} &   -T_x \\
 *                 &   &                            &   -T_y \\
 *                 &   &                            &   -T_z
 *   \end{array} \right)
 *   \pmatrix { X \cr Y \cr Z \cr 1 }
 *
 *   \f]
 *   \f[
 *
 *   \pmatrix { x \cr y \cr 1 } =
 *      \pmatrix{
 *       f \over k &       0    & I_x\cr
 *           0     &  f \over k & I_y\cr
 *           0     &       0    & 1  \cr
 *       }
 *       \pmatrix { u \over t \cr v \over t  \cr 1 }
 *
 *    \f]

 *
 *
 **/
class LocationData
{
public:
    Vector3dd position;
    Quaternion orientation;

    LocationData(
            Vector3dd position = Vector3dd(0.0, 0.0, 1.0),
            Quaternion orientation = Quaternion::Identity()) :
        position(position),
        orientation(orientation)
    {
    }

    Vector3dd project(const Vector3dd &pt)
    {
        return orientation * (pt - position);
    }

    Vector3dd worldToCam(const Vector3dd &pt)
    {
        return project(pt);
    }

    Vector3dd camToWorld(const Vector3dd &pt)
    {
        return orientation.conjugated() * pt + position;
    }

    /**
     *    If we want to transform the world, let's see how camera model will evolve.
     *
     *    X' =  A * X + b
     *    Camera position is an odinary point, it will change accordingly.
     *    Camera rotation had used to transform Zaxis to the main optical axis.
     *    In a new world Zasix has shifted to new position, so new camera matrix should first undo this shift,
     *    and then apply the old transform;
     *
     *    R' = R * A^-1
     *
     **/
    void transform(const Quaternion &rotate, const Vector3dd &translate)
    {
        position    = rotate * position + translate;
        orientation = orientation ^ rotate.conjugated();
    }


    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(position,    Vector3dd(0.0, 0.0, -1.0), "position");
        visitor.visit(orientation, Quaternion::Identity()   , "orientation");
    }

};

/**
 * XXX: We already have intrinsics class somewhere (CameraIntrinsicsLegacy), but
 *      it is not full enough to hold abstract projective pin-hole model (e.g. skewed/non-rectangular)
 *      So this one is now the one to use
 *
 *
 *  TODO: The idea is that if we merge distorsion calibration WITH extrinsics/intrinsics
 *        calibration, then this method will project point using forward distorsion map
 *
 **/

struct PinholeCameraIntrinsics
{
    const static int DEFAULT_SIZE_X = 2592;
    const static int DEFAULT_SIZE_Y = 1944;


    Vector2dd focal;            /**< Focal length (in px) in two directions */
    Vector2dd principal;        /**< Principal point of optical axis on image plane (in pixel). Usually center of imager */
    double skew;                /**< Skew parameter to compensate for optical axis tilt */
    Vector2dd size;             /**< Imager resolution (in pixel) */


    PinholeCameraIntrinsics(
            double fx = 1.0,
            double fy = 1.0,
            double cx = DEFAULT_SIZE_X / 2.0,
            double cy = DEFAULT_SIZE_Y / 2.0,
            double skew = 0.0,
            Vector2dd size = Vector2dd(DEFAULT_SIZE_X, DEFAULT_SIZE_Y)) :
        focal      (fx, fy),
        principal  (cx, cy),
        skew       (skew),
        size       (size)
    {
    }

    /**
     * This actually doesn't differ from matrix multiplication, just is a little bit more lightweight
     *
     **/
    Vector2dd project(const Vector3dd &p)
    {
        Vector2dd result = (focal * p.xy() + Vector2dd(skew * p.y(), 0.0)) / p.z() + principal;
        return result;
    }

    Vector3dd reverse(const Vector2dd &p)
    {
        Vector2dd result;
        result.x() = p.x() / focal.x() + skew / (focal.x() * focal.y()) * p.y() + ( principal.y() * skew / focal.y() - principal.x()) / focal.x();
        result.y() = p.y() / focal.y() - principal.y() / focal.y();
        return Vector3dd(result.x(), result.y(), 1.0);

    }

    explicit operator Matrix33() const
    {
        return getKMatrix33();
    }

    Matrix44 getKMatrix()  const;
    Matrix44 getInvKMatrix()  const;

    Matrix33 getKMatrix33()  const;
    Matrix33 getInvKMatrix33()  const;

    double getVFov() const;
    double getHFov() const;

    double getAspect() const
    {
        return size.y() / size.x();
    }

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        static const Vector2dd DEFAULT_SIZE(DEFAULT_SIZE_X, DEFAULT_SIZE_Y);

        visitor.visit(focal.x()    , 1.0                 , "fx"  );
        visitor.visit(focal.y()    , 1.0                 , "fy"  );
        visitor.visit(principal.x(), DEFAULT_SIZE_X / 2.0, "cx"  );
        visitor.visit(principal.y(), DEFAULT_SIZE_Y / 2.0, "cy"  );
        visitor.visit(skew         , 0.0                 , "skew");
        visitor.visit(size         , DEFAULT_SIZE        , "size");
    }


};

class CameraModel : public ScenePart
{
public:
    /**/
    PinholeCameraIntrinsics intrinsics;
    /**/
    /**/
    LensDistortionModelParameters distortion;

public:
    LocationData extrinsics;
    /* cache for rotation should be introduced. First of all it is faster... */
    //Matrix33 rotMatrix;
public:
    CameraModel() {}

    CameraModel(
            const PinholeCameraIntrinsics &_intrinsics,
            const LocationData &_extrinsics = LocationData(),
            const LensDistortionModelParameters &_distortion = LensDistortionModelParameters()
    ) :
        intrinsics(_intrinsics),
        distortion(_distortion),
        extrinsics(_extrinsics)
    {}


    Vector2dd project(const Vector3dd &pt)
    {
        return intrinsics.project(extrinsics.project(pt));
    }


    Ray3d rayFromPixel(const Vector2dd &point);
    Ray3d rayFromCenter();

    Vector3dd forwardDirection() const;
    Vector3dd topDirection() const;
    Vector3dd rightDirection() const;

    Matrix33  getRotationMatrix() const;
    Matrix44  getCameraMatrix() const;
    Vector3dd getCameraTVector() const;

    ConvexPolyhedron getViewport(const Vector2dd &p1, const Vector2dd &p2);


    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(intrinsics, PinholeCameraIntrinsics()      , "intrinsics");
        visitor.visit(extrinsics, LocationData()                 , "extrinsics");
        visitor.visit(distortion, LensDistortionModelParameters(), "distortion");
    }
};


} // namespace corecvs

#endif // CALIBRATIONCAMERA_H
