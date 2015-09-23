#ifndef CALIBRATION_PHOTOSTATION
#define CALIBRATION_PHOTOSTATION

/**
 * \file calibrationPhotostation.h
 * \ingroup cppcorefiles
 *
 **/

#include <type_traits>

#include "calibrationCamera.h"
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

} // namespace corecvs

template<>
struct is_bitmask<CameraConstraints> : std::true_type {};

namespace corecvs {


/**
 *   See CalibrationScene for more data on ownership of the objectes in structure
 **/
struct Photostation
{
    std::vector<CameraModel> cameras;
    LocationData location;

    Photostation(){}

    Photostation(
        const std::vector<CameraModel> & _cameras,
        const LocationData &_location = LocationData()
    ) :
        cameras(_cameras),
        location(_location)
    {}

    Vector2dd project(const Vector3dd &pt, int cam)
    {
        return cameras[cam].project(pt);
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
        for (CameraModel &cam: ps.cameras)
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

#endif // CALIBRATION_PHOTOSTATION
