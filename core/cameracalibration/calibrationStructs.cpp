#include "calibrationStructs.h"

namespace corecvs {

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
Matrix44 CameraIntrinsics::getKMatrix() const
{
    return Matrix44 (getKMatrix33());
}

/**
 *  Returns inverse of K matrix.
 *  For K matrix read CameraIntrinsics::getKMatrix()
 **/
Matrix44 CameraIntrinsics::getInvKMatrix() const
{

    return Matrix44( getInvKMatrix33());
}

Matrix33 CameraIntrinsics::getKMatrix33() const
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
 *      \frac{1}{fx} & −\frac{s}{fx\,fy} & \frac{\frac{cy\,s}{fy}−cx}{fx}\cr
 *            0      & \frac{1}{fy}      & −\frac{cy}{fy}                \cr
 *            0      &         0         &        1                      }
 * \f]
 *
 **/

Matrix33 CameraIntrinsics::getInvKMatrix33() const
{
    Vector2dd invF = Vector2dd(1.0, 1.0) / focal;

    return Matrix33 (
       invF.x() , - skew * invF.x() * invF.y() ,   ( principal.y() * skew * invF.y() - principal.x()) * invF.x(),
          0.0   ,          invF.y()            ,                                      -principal.y()  * invF.y(),
          0.0   ,            0.0               ,                             1.0
    );
}

double CameraIntrinsics::getVFov() const
{
    return atan((size.y() / 2.0) / focal.y()) * 2.0;
}

double CameraIntrinsics::getHFov() const
{
    return atan((size.x() / 2.0) / focal.x()) * 2.0;
}



} // namespace corecvs
