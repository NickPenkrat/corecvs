#ifndef CALIBRATIONLOCATION_H
#define CALIBRATIONLOCATION_H

#include "vector2d.h"
#include "vector3d.h"
#include "quaternion.h"
#include "matrix44.h"
#include "line.h"

namespace corecvs {

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

    Ray3d relativeRay(const Vector3dd &p)
    {
        return Ray3d(position, worldToCam(p));
    }

    /**
     *    If we want to transform the world, let's see how camera model will evolve.
     *
     *    X' =  A * X + b
     *    Camera position is an odinary point, it will change accordingly.
     *    Camera rotation had used to transform Zaxis to the main optical axis.
     *    In a new world Zaxis has shifted to new position, so new camera matrix should first undo this shift,
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

    void transform(const LocationData &outerTransform)
    {
        transform(outerTransform.orientation, outerTransform.position);
    }



    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(position,    Vector3dd(0.0, 0.0, -1.0), "position");
        visitor.visit(orientation, Quaternion::Identity()   , "orientation");
    }

    /* Pretty print */
    void prettyPrint (ostream &out = cout);
    void prettyPrint1(ostream &out = cout);


};

} // namespace corecvs

#endif // CALIBRATIONLOCATION_H
