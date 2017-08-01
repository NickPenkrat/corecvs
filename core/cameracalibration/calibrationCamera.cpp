#include "calibrationCamera.h"

#include <array>

#include "abstractPainter.h"
#include "rgb24Buffer.h"
#include "bmpLoader.h"

namespace corecvs {

int ScenePart::OBJECT_COUNT = 0;


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
    Matrix33 K1 =       intrinsics.getKMatrix33();
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

PlaneFrame CameraModel::getVirtualScreen(double distance) const
{
    Vector3dd topLeft     = Vector3dd(              0,               0,  1) * distance;
    Vector3dd topRight    = Vector3dd( intrinsics.w(),               0,  1) * distance;
    Vector3dd bottomLeft  = Vector3dd(              0,  intrinsics.h(),  1) * distance;

    PlaneFrame toReturn;

    Matrix33 invK = intrinsics.getInvKMatrix33();
    toReturn.p1  = extrinsics.camToWorld(invK * topLeft);
    Vector3dd d1 = extrinsics.camToWorld(invK * topRight  ) - toReturn.p1;
    Vector3dd d2 = extrinsics.camToWorld(invK * bottomLeft) - toReturn.p1;

    toReturn.e1 = (d1 / intrinsics.w() );
    toReturn.e2 = (d2 / intrinsics.h() );

    return toReturn;
}

Polygon removeDuplicateVertices(Polygon& polygon)
{
    Polygon filteredPolygon;
    for (auto& vertex : polygon)
    {
        if (filteredPolygon.end() == std::find_if(filteredPolygon.begin(), filteredPolygon.end(), [&] ( const corecvs::Vector2dd& p ) -> bool { return (p - vertex).l2Metric() < 1.e-9; }))
            filteredPolygon.push_back(vertex);
    }

    return filteredPolygon;
}

Polygon CameraModel::projectViewport(const CameraModel &right) const
{
#if 0 /* We use a shortcut here */
    vector<Vector4dd> pyramid = right.getCameraViewportPyramid();

    Matrix44 T = getCameraMatrix();

    vector<Vector3dd> pyramidP;
    Polygon pyramidPSimple;


    for (size_t i = 0; i < pyramid.size(); i++)
    {
        Vector4dd out = T * pyramid[i];
        Vector3dd pos1 = Vector3dd( out[0], out[1], out[3]) / out[2];
        pyramidP.push_back(pos1);
        pyramidPSimple.push_back(Vector2dd( out[0], out[1]) / out[2]);
    }

#if 1
    RGB24Buffer *buffer  = new RGB24Buffer(intrinsics.h(), intrinsics.w(), RGBColor::Black());
    AbstractPainter<RGB24Buffer> painter(buffer);
    painter.drawPolygon(pyramidPSimple, RGBColor::Magenta());
    BMPLoader().save("debug.bmp", buffer);
#endif
#endif

    std::vector<Ray3d> rays;
    const int SIDE_STEPS = 100;
    rays.reserve(SIDE_STEPS * 4);

    Vector2dd p1 = Vector2dd::Zero();
    Vector2dd p3 = right.intrinsics.size;
    Vector2dd p2 = Vector2dd(p3.x(), p1.y());
    Vector2dd p4 = Vector2dd(p1.x(), p3.y());

    Ray3d  baseRays[] =
    {
        right.rayFromPixel(p1), right.rayFromPixel(p2), right.rayFromPixel(p3), right.rayFromPixel(p4)
    };

    for (size_t rayId = 0; rayId < CORE_COUNT_OF(baseRays); rayId++ )
    {
        for (int i = 0; i < SIDE_STEPS; i++)
        {
            Ray3d r1 = baseRays[rayId];
            Ray3d r2 = baseRays[(rayId + 1) % CORE_COUNT_OF(baseRays)];
            rays.push_back(Ray3d(lerp(r1.direction(), r2.direction(), i, 0.0, SIDE_STEPS), r1.origin() ));
        }
    }

    /* ==== */
    ConvexPolyhedron viewport = getCameraViewport();
    Matrix44 T = getCameraMatrix();

    //cout << "Ray" << ray << endl;

    /* We go with ray analysis instead of essential matrix beacause it possibly gives
     * more semanticly valuable info
     */

    vector<Vector2dd> points;
    for (size_t rayId = 0; rayId < rays.size(); rayId++ )
    {
        Ray3d &ray = rays[rayId];
        double t1 = 0;
        double t2 = 0;
        bool hasIntersection = viewport.intersectWith(ray, t1, t2);
        if (hasIntersection)
        {
            if (t1 < 0.0) t1 = 0.0;

            FixedVector<double, 4> out1 = (T * ray.getProjectivePoint(t1));
            FixedVector<double, 4> out2 = (T * ray.getProjectivePoint(t2));
            Vector2dd pos1 = Vector2dd( out1[0], out1[1]) / out1[2];
            Vector2dd pos2 = Vector2dd( out2[0], out2[1]) / out2[2];
            points.push_back(pos1);
            points.push_back(pos2);
        }
    }

    return removeDuplicateVertices(ConvexHull::GrahamScan(points));
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
 * given pixel. This method ignores distortion.
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

Matrix44 CameraModel::getPositionMatrix() const
{
    return Matrix44(getRotationMatrix()) * Matrix44::Shift(-extrinsics.position);
}

Vector3dd CameraModel::getCameraTVector() const
{
    return extrinsics.orientation * (-extrinsics.position);
}

ConvexPolyhedron CameraModel::getViewport(const Vector2dd &p1, const Vector2dd &p3) const
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

ConvexPolyhedron  CameraModel::getCameraViewport() const
{
    return getViewport(Vector2dd::Zero(), intrinsics.size);
}

vector<GenericTriangle<Vector4dd> > CameraModel::getCameraViewportSides() const
{
    vector<GenericTriangle<Vector4dd> > sides;

    Vector2dd p1 = Vector2dd::Zero();
    Vector2dd p3 = intrinsics.size;
    Vector2dd p2 = Vector2dd(p3.x(), p1.y());
    Vector2dd p4 = Vector2dd(p1.x(), p3.y());

    Vector4dd position(extrinsics.position, 1.0);
    Vector4dd d1(extrinsics.camToWorld(intrinsics.reverse(p1)), 0.0);
    Vector4dd d2(extrinsics.camToWorld(intrinsics.reverse(p2)), 0.0);
    Vector4dd d3(extrinsics.camToWorld(intrinsics.reverse(p3)), 0.0);
    Vector4dd d4(extrinsics.camToWorld(intrinsics.reverse(p4)), 0.0);

    sides.push_back(GenericTriangle<Vector4dd>(position, d1, d2));
    sides.push_back(GenericTriangle<Vector4dd>(position, d2, d3));
    sides.push_back(GenericTriangle<Vector4dd>(position, d3, d4));
    sides.push_back(GenericTriangle<Vector4dd>(position, d4, d1));
    return sides;

}

vector<Vector4dd> CameraModel::getCameraViewportPyramid() const
{
    vector<Vector4dd> pyramid;

    Vector2dd p1 = Vector2dd::Zero();
    Vector2dd p3 = intrinsics.size;
    Vector2dd p2 = Vector2dd(p3.x(), p1.y());
    Vector2dd p4 = Vector2dd(p1.x(), p3.y());

    Vector4dd position(extrinsics.position, 1.0);
    Vector4dd d1(extrinsics.camToWorld(intrinsics.reverse(p1)), 0.0);
    Vector4dd d2(extrinsics.camToWorld(intrinsics.reverse(p2)), 0.0);
    Vector4dd d3(extrinsics.camToWorld(intrinsics.reverse(p3)), 0.0);
    Vector4dd d4(extrinsics.camToWorld(intrinsics.reverse(p4)), 0.0);

    pyramid.push_back(position);
    pyramid.push_back(d1);
    pyramid.push_back(d2);
    pyramid.push_back(d3);
    pyramid.push_back(d4);

    return pyramid;
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
