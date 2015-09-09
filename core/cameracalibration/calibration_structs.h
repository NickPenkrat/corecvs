#ifndef CALIBRATION_STRUCTS
#define CALIBRATION_STRUCTS

#include <type_traits>

#include "quaternion.h"
#include "matrix33.h"
#include "vector3d.h"
#include "vector2d.h"
#include "typesafeBitmaskEnums.h"
#include "lensDistortionModelParameters.h"
#include "mesh3d.h"
#include "selectableGeometryFeatures.h"

enum class CameraConstraints 
{
    NONE           =  0,
    ZERO_SKEW      =  1, // This forces skew to zero, but not locks it to zero during non-linear optimization
    LOCK_SKEW      =  2, // This one locks skew, but not forces it to zero
    EQUAL_FOCAL    =  4, // Makes fx = fy in non-linear phase
    LOCK_FOCAL     =  8, // Locks fx and fy
    LOCK_PRINCIPAL = 16, // Locks cx and cy
    UNLOCK_YSCALE  = 32  // Unlock Y scale of pattern. This is dangerous if you are not sure what are you doing
};

template<>
struct is_bitmask<CameraConstraints> : std::true_type {};

struct LocationData
{
    LocationData(corecvs::Vector3dd position = corecvs::Vector3dd(0.0, 0.0, 1.0), corecvs::Quaternion orientation = corecvs::Quaternion(0.0, 0.0, 0.0, 1.0)) : position(position), orientation(orientation)
    {
    }

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(position, corecvs::Vector3dd(0.0, 0.0, -1.0), "position");
        visitor.visit(orientation, corecvs::Quaternion(0.0, 0.0, 0.0, 1.0), "orientation");
    }

	corecvs::Vector3dd position;
	corecvs::Quaternion orientation;
};

// XXX: We already have intrinsics class somewhere (CameraIntrinsics), but
//      it is not full enough to hold abstract projective pin-hole model (e.g.
//      skewed/non-rectangular)
struct CameraIntrinsics_
{
    CameraIntrinsics_(double fx = 1.0, double fy = 1.0, double cx = 0.0, double cy = 0.0, double skew = 0.0, corecvs::Vector2dd size = corecvs::Vector2dd(2592.0, 1944.0)) : fx(fx), fy(fy), cx(cx), cy(cy), skew(skew), size(size)
    {
    }
 
    // TODO: The idea is that if we merge distorsion calibration WITH extrinsics/intrinsics
    //       calibration, then this method will project point using forward distorsion map
    corecvs::Vector2dd project(const corecvs::Vector3dd &pt)
    {
        double normalizer = pt[2];
        return corecvs::Vector2dd(fx * pt[0] + skew * pt[1] + cx * pt[2], fy * pt[1] + cy * pt[2]) / normalizer;
    }
	
	explicit operator corecvs::Matrix33() const
    {
        return corecvs::Matrix33(fx, skew, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    }

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(fx, 1.0, "fx");
        visitor.visit(fy, 1.0, "fy");
        visitor.visit(cx, 1.0, "cx");
        visitor.visit(cy, 1.0, "cy");
        visitor.visit(skew, 1.0, "skew");
        visitor.visit(size, corecvs::Vector2dd(2592.0, 1944.0), "size");
    }

	double fx, fy, cx, cy, skew;
    corecvs::Vector2dd size;
};

struct Camera_
{
    corecvs::Vector2dd project(const corecvs::Vector3dd &pt)
    {
        return intrinsics.project(extrinsics.orientation * (pt - extrinsics.position));
    }

    CameraIntrinsics_ intrinsics;
	LocationData extrinsics;
    LensDistortionModelParameters distortion;

	template<class VisitorType>
	void accept(VisitorType &visitor)
    {
        visitor.visit(intrinsics, CameraIntrinsics_(), "intrinsics");
        visitor.visit(extrinsics, LocationData(), "extrinsics");
        visitor.visit(distortion, LensDistortionModelParameters(), "distortion");
    }
};

struct Photostation
{
    corecvs::Vector2dd project(const corecvs::Vector3dd &pt, int cam)
    {
        return cameras[cam].project(location.orientation * (pt - location.position));
    }

    corecvs::Mesh3D drawPly(corecvs::ObservationList &list, double scale = 50.0)
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
        Mesh3D mesh;
        mesh.switchColor(true);


        auto cs = ps.location.position;
        auto qs = ps.location.orientation.conjugated();

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

            auto cc = cam.extrinsics.position;
            auto qc = cam.extrinsics.orientation.conjugated();
            auto A = ((corecvs::Matrix33)cam.intrinsics).inv();

            mesh.currentColor = colors[color = (color + 1) % 6];

            for (int i = 0; i < NSC; ++i)
            {
                auto v1 = qs * (qc * (A * pts[i * 2]) + cc) + cs;
                auto v2 = qs * (qc * (A * pts[i * 2 + 1]) + cc) + cs;

                mesh.addLine(v1, v2);
            }

            auto ppv = qs * (qc * (A * Vector3dd(cam.intrinsics.cx, cam.intrinsics.cy, 1) * scale) + cc) + cs;

            mesh.addLine(ppv, qs * (qc * (A * center) + cc) + cs);
        }

        mesh.currentColor = RGBColor(~0u);
        for (auto& pt: list)
        {
            mesh.addPoint(pt.point);
        }

        return mesh;

    }

	std::vector<Camera_> cameras;
	LocationData location;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(cameras, "cameras");
        visitor.visit(location, LocationData(), "location");
    }
};

typedef std::vector<std::pair<corecvs::Vector2dd, corecvs::Vector3dd>> PatternPoints3d;
typedef std::vector<PatternPoints3d> MultiCameraPatternPoints;


#endif
