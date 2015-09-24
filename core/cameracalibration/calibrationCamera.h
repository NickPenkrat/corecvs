#ifndef CALIBRATIONCAMERA_H
#define CALIBRATIONCAMERA_H

#include <tbb/tbb_machine.h>
#include <unordered_map>

#include "line.h"
#include "convexPolyhedron.h"
#include "lensDistortionModelParameters.h"
#include "calibrationLocation.h"

/* Future derived */
#include "rgb24Buffer.h"


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
    {}

    PinholeCameraIntrinsics(Vector2dd resolution, double hfov);


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
    /* This should be moved to the derived class */
    RGB24Buffer *image;
    std::string fileMame;


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

    bool isVisible(Vector2dd &point)
    {
        return point.isInRect(Vector2dd(0.0,0.0), intrinsics.size);
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
