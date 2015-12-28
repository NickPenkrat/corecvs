#ifndef CALIBRATION_CAMERA_H
#define CALIBRATION_CAMERA_H

#include <unordered_map>

#include "calibrationLocation.h"  // LocationData
#include "lensDistortionModelParameters.h"
#include "line.h"
#include "convexPolyhedron.h"
#include "pointObservation.h"

/* Future derived */
//#include "rgb24Buffer.h"

namespace corecvs {

/*class RGB24Buffer;*/
class CalibrationScene;

/**
 * This class is so far just a common base for all objects in scene heap.
 * Should bring this to other file
 **/
class ScenePart {
public:
    /* No particular reason for this, except to encourage leak checks */
    static int OBJECT_COUNT;

    CalibrationScene *ownerScene;

    /* We could have copy constructors and stuff... but so far this is enough */
    ScenePart(CalibrationScene * owner = NULL) :
        ownerScene(owner)
    {
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
    double    skew;             /**< Skew parameter to compensate for optical axis tilt */
    Vector2dd size;             /**< Imager resolution (in pixel) */
    Vector2dd distortedSize;    /**< Source image resolution (FIXME: probably should move it somewhere) */


    PinholeCameraIntrinsics(
            double fx = 1.0,
            double fy = 1.0,
            double cx = DEFAULT_SIZE_X / 2.0,
            double cy = DEFAULT_SIZE_Y / 2.0,
            double skew = 0.0,
            Vector2dd size = Vector2dd(DEFAULT_SIZE_X, DEFAULT_SIZE_Y),
            Vector2dd distortedSize = Vector2dd(DEFAULT_SIZE_X, DEFAULT_SIZE_Y))
      : focal         (fx, fy)
      , principal     (cx, cy)
      , skew          (skew)
      , size          (size)
      , distortedSize (distortedSize)
    {}

    PinholeCameraIntrinsics(Vector2dd resolution, double hfov);


    /**
     * This actually doesn't differ from matrix multiplication, just is a little bit more lightweight
     *
     **/
    Vector2dd project(const Vector3dd &p) const
    {
        Vector2dd result = (focal * p.xy() + Vector2dd(skew * p.y(), 0.0)) / p.z() + principal;
        return result;
    }

    Vector3dd reverse(const Vector2dd &p) const
    {
        Vector2dd result;
        result.x() = p.x() / focal.x() + skew / (focal.x() * focal.y()) * p.y() + ( principal.y() * skew / focal.y() - principal.x()) / focal.x();
        result.y() = p.y() / focal.y() - principal.y() / focal.y();
        return Vector3dd(result.x(), result.y(), 1.0);
    }

    bool isVisible(const Vector3dd &p) const
    {
        if (p[2] <= 0.0) {
            return false;
        }
        Vector2dd proj = project(p);

        if (!proj.isInRect(Vector2dd(0.0, 0.0), size))
        {
            return false;
        }
        return true;
    }

    explicit operator Matrix33() const  { return getKMatrix33(); }

    Matrix44 getKMatrix() const;
    Matrix44 getInvKMatrix() const;

    Matrix33 getKMatrix33() const;
    Matrix33 getInvKMatrix33() const;

    double   getVFov() const;
    double   getHFov() const;

    double   getAspect() const          { return size.y() / size.x(); }

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
        visitor.visit(distortedSize, DEFAULT_SIZE        , "distortedSize");
    }

    /* Helper pseudonim getters */
    double  h() const    { return size.y();      }
    double  w() const    { return size.x();      }
    double cx() const    { return principal.x(); }
    double cy() const    { return principal.y(); }
};

class Photostation;

class CameraModel /*: public ScenePart*/
{
public:
    /**/
    PinholeCameraIntrinsics         intrinsics;
    /**/
    LensDistortionModelParameters   distortion;
    /**/
    CameraLocationData              extrinsics;

    /* cache for rotation should be introduced. First of all it is faster... */
    //Matrix33 rotMatrix;

public:
  /*  Photostation   *station;*/

    /* This should be moved to the derived class */
    /*RGB24Buffer    *image;*/
    std::string     nameId;

public:
    CameraModel(/*CalibrationScene * owner = NULL*/) /*:
        ScenePart(owner)*/
    {}

    CameraModel(
            const PinholeCameraIntrinsics &_intrinsics,
            const CameraLocationData &_extrinsics = CameraLocationData(),
            const LensDistortionModelParameters &_distortion = LensDistortionModelParameters())
      : intrinsics(_intrinsics)
      , distortion(_distortion)
      , extrinsics(_extrinsics)
    {}


    Vector2dd project(const Vector3dd &p) const
    {
        return intrinsics.project(extrinsics.project(p));
    }

    Vector2dd reprojectionError(PointObservation &observation)
    {
        return observation.projection - project(observation.point);
    }

    /**
     * Return a direction in camera corrdinate frame passing though a point p.
     *
     * This only uses extrinsics and provides the way to intercept the projection process before
     * intrinsics application take action
     *
     * \param p
     **/
    Vector3dd dirToPoint(const Vector3dd &p)
    {
        return extrinsics.project(p);
    }

    /**
     * Only checks for the fact that point belongs to viewport.
     * If you are projecting 3d point you should be sure that point is in front
     **/
    bool isVisible(Vector2dd &point)
    {
        return point.isInRect(Vector2dd(0.0,0.0), intrinsics.size);
    }

    /**
     * Checks full visibility of 3d point
     **/
    bool isVisible(const Vector3dd &pt) const
    {
        return intrinsics.isVisible(extrinsics.project(pt));
    }

    bool isInFront(Vector3dd &pt)
    {
        return ((pt - extrinsics.position) & forwardDirection()) > 0;
    }

    Ray3d               rayFromPixel(const Vector2dd &point);
    Ray3d               rayFromCenter();

    Vector3dd           forwardDirection() const;
    Vector3dd           topDirection() const;
    Vector3dd           rightDirection() const;

    Matrix33            getRotationMatrix() const;
    Matrix44            getCameraMatrix() const;
    Vector3dd           getCameraTVector() const;

    ConvexPolyhedron    getViewport(const Vector2dd &p1, const Vector2dd &p2);


    void copyModelFrom(const CameraModel &other) {
        intrinsics = other.intrinsics;
        distortion = other.distortion;
        extrinsics = other.extrinsics;
    }

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(intrinsics, PinholeCameraIntrinsics()      , "intrinsics");
        visitor.visit(extrinsics, CameraLocationData()           , "extrinsics");
        visitor.visit(distortion, LensDistortionModelParameters(), "distortion");
        visitor.visit(nameId,     std::string("")                , "nameId"    );
    }

    void prettyPrint(std::ostream &out = cout);
};


} // namespace corecvs

#endif // CALIBRATION_CAMERA_H
