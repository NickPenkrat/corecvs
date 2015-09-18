#ifndef CALIBRATION_STRUCTS
#define CALIBRATION_STRUCTS

#include <type_traits>

#include "quaternion.h"
#include "matrix33.h"
#include "matrix44.h"
#include "vector3d.h"
#include "vector2d.h"
#include "typesafeBitmaskEnums.h"
#include "lensDistortionModelParameters.h"
#include "mesh3d.h"
#include "selectableGeometryFeatures.h"

namespace corecvs {

enum class CameraConstraints 
{
    NONE           =  0x00,
    ZERO_SKEW      =  0x01, // This forces skew to zero, but not locks it to zero during non-linear optimization
    LOCK_SKEW      =  0x02, // This one locks skew, but not forces it to zero
    EQUAL_FOCAL    =  0x04, // Makes fx = fy in non-linear phase
    LOCK_FOCAL     =  0x08, // Locks fx and fy
    LOCK_PRINCIPAL =  0x10, // Locks cx and cy
    UNLOCK_YSCALE  =  0x20  // Unlock Y scale of pattern. This is dangerous if you are not sure what are you doing
};



}

template<>
struct is_bitmask<CameraConstraints> : std::true_type {};

namespace corecvs {

struct LocationData
{
    LocationData(Vector3dd position = Vector3dd(0.0, 0.0, 1.0), Quaternion orientation = Quaternion(0.0, 0.0, 0.0, 1.0)) : position(position), orientation(orientation)
    {
    }

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(position, Vector3dd(0.0, 0.0, -1.0), "position");
        visitor.visit(orientation, Quaternion(0.0, 0.0, 0.0, 1.0), "orientation");
    }

    Vector3dd position;
    Quaternion orientation;
};

/**
 * XXX: We already have intrinsics class somewhere (CameraIntrinsicsLegacy), but
 *      it is not full enough to hold abstract projective pin-hole model (e.g. skewed/non-rectangular)
 *      So this one is now the one to use
 */

struct CameraIntrinsics
{
    const static int DEFAULT_SIZE_X = 2592;
    const static int DEFAULT_SIZE_Y = 1944;


    Vector2dd focal;            /**< Focal length (in px) in two directions */
    Vector2dd principal;        /**< Principal point of optical axis on image plane (in pixel). Usually center of imager */
    double skew;
    Vector2dd size;    /**< Imager resolution (in pixel) */


    CameraIntrinsics(
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
 
    // TODO: The idea is that if we merge distorsion calibration WITH extrinsics/intrinsics
    //       calibration, then this method will project point using forward distorsion map
    Vector2dd project(const Vector3dd &p)
    {
        Vector2dd result = (focal * p.xy() + Vector2dd(skew * p.y(), 0.0)) / p.z() + principal;
        return result;
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

struct Camera_
{
    CameraIntrinsics intrinsics;
    LocationData extrinsics;
    LensDistortionModelParameters distortion;

    Vector2dd project(const Vector3dd &pt)
    {
        return intrinsics.project(extrinsics.orientation * (pt - extrinsics.position));
    }

	template<class VisitorType>
	void accept(VisitorType &visitor)
    {
        visitor.visit(intrinsics, CameraIntrinsics()             , "intrinsics");
        visitor.visit(extrinsics, LocationData()                 , "extrinsics");
        visitor.visit(distortion, LensDistortionModelParameters(), "distortion");
    }
};

struct Photostation
{
    std::vector<Camera_> cameras;
    LocationData location;


    Vector2dd project(const Vector3dd &pt, int cam)
    {
        return cameras[cam].project(location.orientation * (pt - location.position));
    }

    void drawPly(Mesh3D &mesh, double scale = 50.0)
    {
        auto& ps = *this;
        // Colorblind-safe palette
        RGBColor colors[] =
        {
            RGBColor(0x762a83u),
            RGBColor(0xaf8dc3u),
            RGBColor(0xe7d4e8u),
            RGBColor(0xd9f0d3u),
            RGBColor(0x7fbf7bu),
            RGBColor(0x1b7837u)
        };
        auto cs = ps.location.position;
        auto qs = ps.location.orientation.conjugated();
        std::cout << qs << std::endl;

        int color = 0;
        for (auto& cam: ps.cameras)
        {
            double IW = cam.intrinsics.size[0];
            double IH = cam.intrinsics.size[1];
            const int NSC = 9;
            Vector3dd center      = Vector3dd( 0,  0,  0),
                    center2     = Vector3dd( 0,  0,  1) * scale,
                    topLeft     = Vector3dd( 0,  0,  1) * scale,
                    topRight    = Vector3dd(IW,  0,  1) * scale,
                    bottomRight = Vector3dd(IW, IH,  1) * scale,
                    bottomLeft  = Vector3dd( 0, IH,  1) * scale;
            Vector3dd pts[NSC * 2] =
            {
                center, center2,
                center, topLeft,
                center, topRight,
                center, bottomRight,
                center, bottomLeft,
                topLeft, topRight,
                topRight, bottomRight,
                bottomRight, bottomLeft,
                bottomLeft, topLeft,
            };

            auto cc    = cam.extrinsics.position;
            auto qc    = cam.extrinsics.orientation.conjugated();
            Matrix33 A = cam.intrinsics.getInvKMatrix33();

            mesh.currentColor = colors[color = (color + 1) % 6];

            for (int i = 0; i < NSC; ++i)
            {
                auto v1 = qs * (qc * (A * pts[i * 2]) + cc) + cs;
                auto v2 = qs * (qc * (A * pts[i * 2 + 1]) + cc) + cs;

                mesh.addLine(v1, v2);
            }

            auto ppv = qs * (qc * (A.mulBy2dRight(cam.intrinsics.principal) * scale) + cc) + cs;

            mesh.addLine(ppv, qs * (qc * (A * center) + cc) + cs);
        }
    }

    void drawPly(Mesh3D &mesh, ObservationList &list)
    {
        mesh.currentColor = RGBColor(~0u);
        for (auto& pt: list)
        {
            mesh.addPoint(pt.point);
        }

    }

    void drawPly(Mesh3D &mesh, ObservationList &list, double scale)
    {
        drawPly(mesh, list);
        drawPly(mesh, scale);
    }

    Mesh3D drawPly(ObservationList &list, double scale = 50.0)
    {
        Mesh3D mesh;
        mesh.switchColor(true);

        drawPly(mesh, list, scale);

        return mesh;

    }


    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(cameras, "cameras");
        visitor.visit(location, LocationData(), "location");
    }
};

typedef std::vector<std::pair<Vector2dd, Vector3dd>> PatternPoints3d;
typedef std::vector<PatternPoints3d> MultiCameraPatternPoints;

}

#endif
