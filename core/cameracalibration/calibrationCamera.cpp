#include "calibrationCamera.h"

#include <array>

namespace corecvs {

int ScenePart::OBJECT_COUNT = 0;

Matrix44 PinholeCameraIntrinsics::Kf()
{
    return Matrix44(1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0);
}

Matrix44 PinholeCameraIntrinsics::Kfx()
{
    return Matrix44(1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0);
}

Matrix44 PinholeCameraIntrinsics::Kfy()
{
    return Matrix44(0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0);
}

Matrix44 PinholeCameraIntrinsics::Kcx()
{
    return Matrix44(0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0);
}

Matrix44 PinholeCameraIntrinsics::Kcy()
{
    return Matrix44(0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0);
}

Matrix44 PinholeCameraIntrinsics::Kif(const double &f, const double &cx, const double &cy)
{
    auto f2 = f * f;
    auto f21 = 1.0 / f2;
    return Matrix44(-f21, 0.0, cx / f2, 0.0,
                     0.0,-f21, cy / f2, 0.0,
                     0.0, 0.0,     0.0, 0.0,
                     0.0, 0.0,     0.0, 0.0);
}

Matrix44 PinholeCameraIntrinsics::Kifx(const double &fx, const double &cx)
{
    auto f2 = fx * fx;
    auto f21 = 1.0 / f2;
    return Matrix44(-f21, 0.0, cx / f2, 0.0,
                     0.0, 0.0,     0.0, 0.0,
                     0.0, 0.0,     0.0, 0.0,
                     0.0, 0.0,     0.0, 0.0);
}

Matrix44 PinholeCameraIntrinsics::Kify(const double &fy, const double &cy)
{
    auto f2 = fy * fy;
    auto f21 = 1.0 / f2;
    return Matrix44(0.0, 0.0,     0.0, 0.0,
                    0.0,-f21, cy / f2, 0.0,
                    0.0, 0.0,     0.0, 0.0,
                    0.0, 0.0,     0.0, 0.0);
}

Matrix44 PinholeCameraIntrinsics::Kicx(const double &fx)
{
    return Matrix44(0.0, 0.0, -1.0 / fx, 0.0,
                    0.0, 0.0,       0.0, 0.0,
                    0.0, 0.0,       0.0, 0.0,
                    0.0, 0.0,       0.0, 0.0);
}

Matrix44 PinholeCameraIntrinsics::Kicy(const double &fy)
{
    return Matrix44(0.0, 0.0,       0.0, 0.0,
                    0.0, 0.0, -1.0 / fy, 0.0,
                    0.0, 0.0,       0.0, 0.0,
                    0.0, 0.0,       0.0, 0.0);
}



Matrix44 PinholeCameraIntrinsics::NormalizerDiff(const double &x, const double &y, const double &z)
{
    double zz = z * z;
    return Matrix44(1.0 / z,     0.0, -x / zz, 0.0,
                        0.0, 1.0 / z, -y / zz, 0.0,
                        0.0,     0.0,     0.0, 0.0,
                        0.0,     0.0,     0.0, 0.0);
}

Matrix44 PinholeCameraIntrinsics::RayDiffNormalizerDiff(const double &ux, const double &uy, const double &uz)
{
    auto ux2 = ux*ux, uy2 = uy * uy, uz2 = uz * uz,
            uxuy= ux*uy, uxuz= ux * uz, uyuz= uy * uz;
    auto N = ux2 + uy2 + uz2;
    auto N2 = std::sqrt(N);
    auto N32 = N * N2, N21 = 1.0 / N2;
    return Matrix44   (
            -ux2/N32+N21,    -uxuy/N32,    -uxuz/N32, 0.0,
               -uxuy/N32, -uy2/N32+N21,    -uyuz/N32, 0.0,
               -uxuz/N32,    -uyuz/N32, -uz2/N32+N21, 0.0,
                     0.0,          0.0,          0.0, 0.0);
}

Matrix33 CameraModel::Fundamental(const Matrix44 &L, const Matrix44 &R)
{
    const Matrix44 P[2] = {R, L};
    Matrix44 t[3][3];
    int rows[3][3][2][3]
// C++ generally and corecvs particullary are not very cool at clever indexing
#define X1 {0, 1, 2}
#define X2 {0, 2, 0}
#define X3 {0, 0, 1}
#define Y1 {1, 1, 2}
#define Y2 {1, 2, 0}
#define Y3 {1, 0, 1}
     = {
        {{ X1, Y1 }, { X2, Y1 }, { X3, Y1 }},
        {{ X1, Y2 }, { X2, Y2 }, { X3, Y2 }},
        {{ X1, Y3 }, { X2, Y3 }, { X3, Y3 }}
     };
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 2; ++k)
                for (int l = 0; l < 4; ++l)
                {
                    t[i][j].a(k * 2,     l) = P[rows[i][j][k][0]].a(rows[i][j][k][1], l);
                    t[i][j].a(k * 2 + 1, l) = P[rows[i][j][k][0]].a(rows[i][j][k][2], l);
                }
    Matrix33 F;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            F.a(i, j) = t[i][j].det();
    return F;
}

Matrix33 CameraModel::fundamentalTo(const CameraModel &right) const
{
    Matrix33 K1 = intrinsics.getKMatrix33();
    Matrix33 K2 = right.intrinsics.getKMatrix33();
    return K1.inv().transposed() * essentialTo(right) * K2.inv();
}
Matrix33 CameraModel::essentialTo  (const CameraModel &right) const
{
    return (Matrix33)essentialDecomposition(right);
}

EssentialDecomposition CameraModel::essentialDecomposition(const CameraModel &right) const
{
    return CameraModel::ComputeEssentialDecomposition(extrinsics, right.extrinsics);
}

EssentialDecomposition CameraModel::ComputeEssentialDecomposition(const CameraLocationData &thisData, const CameraLocationData &otherData)
{
    Quaternion R =  thisData.orientation ^ otherData.orientation.conjugated();
    Vector3dd  T =  thisData.orientation * (-thisData.position + otherData.position);
    return EssentialDecomposition(R.toMatrix(), T);
}


PinholeCameraIntrinsics::PinholeCameraIntrinsics(Vector2dd resolution, double hfov)
  : principal(resolution / 2.0)
  , skew(0.0)
  , size(resolution)
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
Ray3d CameraModel::rayFromPixel(const Vector2dd &point) const
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

Matrix44 CameraModel::getFrustrumMatrix(double zNear, double zFar) const
{
    /*
    double zDepth = zNear - zFar;
    Matrix44 matrix (
        f / aspect, 0 ,             0          ,            0            ,
        0         , f ,             0          ,            0            ,
        0         , 0 , (zFar + zNear) / zDepth, 2 *zFar * zNear / zDepth,
        0         , 0 ,            -1          ,            0
    );

    */

    if (zNear == -1) {
        zNear = 0.001;
    }

    if (zFar == -1) {
        zFar = 1000;
    }

    double zDepth       = zNear - zFar;
    Vector2dd focal     = intrinsics.focal;
    Vector2dd principal = intrinsics.principal;
    Vector2dd size      = intrinsics.size;

    double skew = intrinsics.skew;

    Matrix44 KF =  Matrix44 (
        focal.x(),   skew   ,          0.0           ,    /*principal.x()*/ 0,
           0.0   , focal.y(),          0.0           ,    /*principal.y()*/ 0,
           0.0   ,    0.0   , (zFar + zNear) / zDepth, 2 * zFar * zNear / zDepth,
           0.0   ,    0.0   ,         -1.0           ,        0.0
    );

    Matrix44 D = /*Matrix44::Shift(0.5, 0.5, 0.0) **/ Matrix44::Diagonal(2.0 / size.x(), 2.0 / size.y(), 1.0, 1.0);

    Matrix44 T = Matrix44(getRotationMatrix()) * Matrix44::Shift(-extrinsics.position);
    cout << "Position Matrix:" << endl;
    cout <<  T;

    cout << "K Matrix:" << endl;
    cout <<  KF;

    cout << "D Matrix:" << endl;
    cout <<  D;

    Matrix44 toReturn = D * KF * T;

    cout << "Result Matrix:" << endl;
    cout <<  toReturn;

    return  toReturn;


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

#ifndef Rect
typedef std::array<corecvs::Vector2dd, 2> Rect;
#endif

void corecvs::CameraModel::estimateUndistortedSize(const DistortionApplicationParameters &applicationParams)
{
    Rect input = { corecvs::Vector2dd(0.0, 0.0), intrinsics.distortedSize }, output;
    Rect outCir, outIns;

    distortion.getCircumscribedImageRect(input[0], input[1], outCir[0], outCir[1]);
    distortion.getInscribedImageRect    (input[0], input[1], outIns[0], outIns[1]);

    output = input;
    corecvs::Vector2dd shift(0.0);

    switch (applicationParams.resizePolicy())
    {
        case DistortionResizePolicy::FORCE_SIZE:
            output = { corecvs::Vector2dd(0.0, 0.0), intrinsics.size };
            shift[0] = distortion.principalX() / intrinsics.distortedSize[0] * output[1][0];
            shift[1] = distortion.principalY() / intrinsics.distortedSize[1] * output[1][1];
            break;
        case DistortionResizePolicy::TO_FIT_RESULT:
            output = outCir;
            shift = -output[0];
            output[1] -= output[0];
            output[0] -= output[0];
            break;
        case DistortionResizePolicy::TO_NO_GAPS:
            output = outIns;
            shift = -output[0];
            output[1] -= output[0];
            output[0] -= output[0];
            break;
        case DistortionResizePolicy::NO_CHANGE:
        default:
            break;
    }

    double w = intrinsics.distortedSize[0];
    double h = intrinsics.distortedSize[1];
    intrinsics.size = output[1] - output[0];
    double newW = intrinsics.size[0];
    double newH = intrinsics.size[1];

    if (applicationParams.adoptScale())
    {
        double aspect = std::max(newW / w, newH / h);
        distortion.setScale(1.0 / aspect);
    }
    else
    {
        /*
         * In order to maintain idempotence of undistorted size estimation
         * one needs to update intrinsics.principal only by shift,
         * because [in the next call with the same DistortionApplicationParameters]
         * shift should be ~0, since it will be eliminated by distortion.mShift*
         */
        intrinsics.principal += shift;
        distortion.mShiftX += shift[0];
        distortion.mShiftY += shift[1];
    }
}


void CameraModel::prettyPrint(std::ostream &out)
{
    PrinterVisitor visitor(3, 3, out);
    cout << "Camera:" << nameId << std::endl;
    accept(visitor);
}


} // namespace corecvs
