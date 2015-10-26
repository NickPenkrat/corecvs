#ifndef CALIBRATIONLOCATION_H
#define CALIBRATIONLOCATION_H

#include "vector2d.h"
#include "vector3d.h"
#include "quaternion.h"
#include "matrix44.h"
#include "line.h"
#include "eulerAngles.h"
#include "affine.h"
#include "printerVisitor.h"

#include "mathUtils.h"

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
 **/
class CameraLocationData
{
public:
    Vector3dd position;
    Quaternion orientation;

    explicit CameraLocationData(
            Vector3dd position = Vector3dd(0.0, 0.0, 1.0),
            Quaternion orientation = Quaternion::Identity()) :
        position(position),
        orientation(orientation)
    {
    }

    /**
     * Helper function that creates a CameraLocationData that acts just as a Affine3DQ
     *
     * Affine
     *    X' = AR * X + AT
     *
     * Cam
     *    X' = CR * (X - CT) = CR * X - CR * CT
     *
     *    AT = - CR * CT
     *
     *    CR = AR
     *    CT = CR^{-1} (- AT)
     *
     *
     **/
    explicit CameraLocationData( const Affine3DQ &transform ) :
        position(transform.rotor.conjugated() * (-transform.shift)),
        orientation(transform.rotor)
    {}

    Affine3DQ toAffine3D() const
    {
        return Affine3DQ(orientation, - (orientation * position));
    }

    Vector3dd project(const Vector3dd &pt) const
    {
        return orientation * (pt - position);
    }

    Vector3dd worldToCam(const Vector3dd &pt) const
    {
        return project(pt);
    }

    Vector3dd camToWorld(const Vector3dd &pt) const
    {
        return orientation.conjugated() * pt + position;
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

    void transform(const Affine3DQ &affine)
    {
        transform(affine.rotor, affine.shift);
    }

    /*void transform(const CameraLocationData &outerTransform)
    {
        transform(outerTransform.orientation, outerTransform.position);
    }*/

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


/**
 *    The classical rotation storage is in format yaw, pitch and roll
 *
 *    Yaw/Athimuth [0..2pi]
 *    Pitch
 *    Roll
 *
 *
 **/
class CameraLocationAngles : public EulerAngles
{
public:
    CameraLocationAngles(double yaw = 0.0, double pitch = 0.0, double roll = 0.0) :
        EulerAngles(yaw, pitch, roll)
    {}

    static CameraLocationAngles FromAngles(double yawDeg, double pitchDeg, double rollDeg)
    {
        return CameraLocationAngles(degToRad(yawDeg), degToRad(pitchDeg), degToRad(rollDeg));
    }

    double  yaw() const
    {
        return alpha;
    }

    void setYaw(double yaw)
    {
        alpha = yaw;
    }

    double  pitch() const
    {
        return beta;
    }

    void setPitch(double pitch)
    {
        beta = pitch;
    }

    double  roll() const
    {
        return gamma;
    }

    void setRoll(double roll)
    {
        gamma = roll;
    }

    Matrix33 toMatrix() const
    {
        return
            Matrix33::RotationZ(roll()) *
            Matrix33::RotationY(yaw()) *
            Matrix33::RotationX(pitch());
    }

    /**
     *  \f[
     *  \pmatrix{ \phi \cr \theta \cr \psi } =
     *
     *  \pmatrix{
     *     atan2  (2(q_0 q_1 + q_2 q_3),1 - 2(q_1^2 + q_2^2)) \cr
     *     arcsin (2(q_0 q_2 - q_3 q_1)) \cr
     *     atan2  (2(q_0 q_3 + q_1 q_2),1 - 2(q_2^2 + q_3^2))
     *  }
     *
     *  \f]
     *
     */
    static CameraLocationAngles FromQuaternion(Quaternion &Q)
    {
        double yaw   = asin  (2.0 * (Q.t() * Q.y() - Q.z() * Q.x()));
        double pitch = atan2 (2.0 * (Q.t() * Q.x() + Q.y() * Q.z()),1.0 - 2.0 * (Q.x() * Q.x() + Q.y() * Q.y()));
        double roll  = atan2 (2.0 * (Q.t() * Q.z() + Q.x() * Q.y()),1.0 - 2.0 * (Q.y() * Q.y() + Q.z() * Q.z()));
        return CameraLocationAngles(yaw, pitch, roll);
    }


    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(alpha, 0.0, "yaw"  );
        visitor.visit(beta , 0.0, "pitch");
        visitor.visit(gamma, 0.0, "roll" );
    }

    CameraLocationAngles toDeg() const {
        return CameraLocationAngles(
                    radToDeg(yaw()),
                    radToDeg(pitch()),
                    radToDeg(roll())
                    );
    }

    CameraLocationAngles toRad() const {
        return CameraLocationAngles(
                    degToRad(yaw()),
                    degToRad(pitch()),
                    degToRad(roll())
                    );
    }

    friend ostream& operator << (ostream &out, CameraLocationAngles &toSave)
    {
        PrinterVisitor printer(out);
        toSave.accept<PrinterVisitor>(printer);
        return out;
    }

    void prettyPrint (ostream &out = cout);

};

} // namespace corecvs

#endif // CALIBRATIONLOCATION_H
