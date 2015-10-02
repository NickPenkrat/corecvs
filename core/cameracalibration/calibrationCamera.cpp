#include "calibrationCamera.h"

namespace corecvs {

int ScenePart::OBJECT_COUNT = 0;


PinholeCameraIntrinsics::PinholeCameraIntrinsics(Vector2dd resolution, double hfov) :
    principal(resolution / 2.0),
    skew(0.0),
    size(resolution)

{
    double ratio = tan(hfov / 2.0);
    double f = (size.x() / 2.0) / ratio;

    focal = Vector2dd(f,f);
}



/**
 *  We will denote it by matrix \f$K\f$.
 *
 *  \f[K =  \pmatrix{
 *       f_x  &   s  & I_x & 0 \cr
 *        0   &  f_y & I_y & 0 \cr
 *        0   &   0  & 1   & 0 \cr
 *        0   &   0  & 0   & 1  }
 * \f]
 *
 *
 * This matrix transform the 3D points form camera coordinate system to
 * image coordinate system
 **/
Matrix44 PinholeCameraIntrinsics::getKMatrix() const
{
    return Matrix44 (getKMatrix33());
}

/**
 *  Returns inverse of K matrix.
 *  For K matrix read CameraIntrinsics::getKMatrix()
 **/
Matrix44 PinholeCameraIntrinsics::getInvKMatrix() const
{

    return Matrix44( getInvKMatrix33());
}

Matrix33 PinholeCameraIntrinsics::getKMatrix33() const
{
    return Matrix33 (
        focal.x(),   skew   , principal.x(),
           0.0   , focal.y(), principal.y(),
           0.0   ,    0.0   ,     1.0
    );
}

/**
 *
 * \f[ K^{-1} = \pmatrix{
 *      \frac{1}{f_x} & {- s} \over {f_x f_y} & {{{c_y s} \over {f_y}} - {c_x}} \over {f_x} \cr
 *            0       &   {1} \over {f_y}      &          {- c_y} \over {f_y}             \cr
 *            0       &         0              &                   1                     }
 * \f]
 *
 **/

Matrix33 PinholeCameraIntrinsics::getInvKMatrix33() const
{
    Vector2dd invF = Vector2dd(1.0, 1.0) / focal;

    return Matrix33 (
       invF.x() , - skew * invF.x() * invF.y() ,   ( principal.y() * skew * invF.y() - principal.x()) * invF.x(),
          0.0   ,          invF.y()            ,                                      -principal.y()  * invF.y(),
          0.0   ,            0.0               ,                             1.0
    );
}

double PinholeCameraIntrinsics::getVFov() const
{
    return atan((size.y() / 2.0) / focal.y()) * 2.0;
}

double PinholeCameraIntrinsics::getHFov() const
{
    return atan((size.x() / 2.0) / focal.x()) * 2.0;
}

/* Camera */

/**
 * Returns a ray in a world coordinate system that originates at the camera position and goes through
 * given pixel
 *
 *  \param point - a point in image coorinates
 *
 **/
Ray3d CameraModel::rayFromPixel(const Vector2dd &point)
{
    Vector3dd direction = intrinsics.reverse(point);
    Ray3d ray(extrinsics.orientation.conjugated() * direction, extrinsics.position);
    return ray;
}

Ray3d CameraModel::rayFromCenter()
{
    return rayFromPixel(intrinsics.principal);
}

Vector3dd CameraModel::forwardDirection() const
{
    //return rotation.column(2);
    return extrinsics.orientation.conjugated() * Vector3dd(0.0, 0.0, 1.0);
}

Vector3dd CameraModel::topDirection() const
{
    return extrinsics.orientation.conjugated() * Vector3dd(0.0, 1.0, 0.0);
}

Vector3dd CameraModel::rightDirection() const
{
    return extrinsics.orientation.conjugated() * Vector3dd(1.0, 0.0, 0.0);
}

Matrix33 CameraModel::getRotationMatrix() const
{
    return extrinsics.orientation.toMatrix();
}


Matrix44 CameraModel::getCameraMatrix() const
{
    return intrinsics.getKMatrix() * Matrix44(getRotationMatrix()) * Matrix44::Shift(-extrinsics.position);
}

Vector3dd CameraModel::getCameraTVector() const
{
    return extrinsics.orientation * (-extrinsics.position);
}

ConvexPolyhedron CameraModel::getViewport(const Vector2dd &p1, const Vector2dd &p3)
{
    ConvexPolyhedron toReturn;
    Vector3dd position = extrinsics.position;

    //Vector3dd direction1((point.x() - optCenter.x()) / focal, (point.y() - optCenter.y()) / focal, 1.0);

    Vector2dd p2 = Vector2dd(p3.x(), p1.y());
    Vector2dd p4 = Vector2dd(p1.x(), p3.y());

    Vector3dd d1 = extrinsics.camToWorld(intrinsics.reverse(p1));
    Vector3dd d2 = extrinsics.camToWorld(intrinsics.reverse(p2));
    Vector3dd d3 = extrinsics.camToWorld(intrinsics.reverse(p3));
    Vector3dd d4 = extrinsics.camToWorld(intrinsics.reverse(p4));


    toReturn.faces.push_back( Plane3d::FromPoints(position, d1, d2) );
    toReturn.faces.push_back( Plane3d::FromPoints(position, d2, d3) );
    toReturn.faces.push_back( Plane3d::FromPoints(position, d3, d4) );
    toReturn.faces.push_back( Plane3d::FromPoints(position, d4, d1) );

    return toReturn;
}


} // namespace corecvs
