#include "calibrationHelpers.h"
#include "mesh3d.h"
#include "calibrationPhotostation.h"
#include "selectableGeometryFeatures.h"

RGBColor CalibrationHelpers::palette[] =
{
    RGBColor(0x762a83u),
    RGBColor(0xaf8dc3u),
    RGBColor(0xe7d4e8u),
    RGBColor(0xd9f0d3u),
    RGBColor(0x7fbf7bu),
    RGBColor(0x1b7837u)
};


void CalibrationHelpers::drawPly(Mesh3D &mesh, Photostation &ps, double scale)
{
    // Colorblind-safe palette
    Vector3dd  cs = ps.location.position;
    Quaternion qs = ps.location.orientation.conjugated();
    std::cout << qs << std::endl;

    int colorId = 0;
    for (CameraModel &cam: ps.cameras)
    {
        double IW = cam.intrinsics.size[0];
        double IH = cam.intrinsics.size[1];
        const int NSC = 9;
        Vector3dd center    = Vector3dd( 0,  0,  0),
                center2     = Vector3dd( 0,  0,  1) * scale,
                topLeft     = Vector3dd( 0,  0,  1) * scale,
                topRight    = Vector3dd(IW,  0,  1) * scale,
                bottomRight = Vector3dd(IW, IH,  1) * scale,
                bottomLeft  = Vector3dd( 0, IH,  1) * scale;
        Vector3dd pts[NSC * 2] =
        {
            center,      center2,
            center,      topLeft,
            center,      topRight,
            center,      bottomRight,
            center,      bottomLeft,
            topLeft,     topRight,
            topRight,    bottomRight,
            bottomRight, bottomLeft,
            bottomLeft,  topLeft,
        };

        Vector3dd  cc  = cam.extrinsics.position;
        Quaternion qc  = cam.extrinsics.orientation.conjugated();
        Matrix33   K   = cam.intrinsics.getInvKMatrix33();

        mesh.currentColor = palette[colorId];
        colorId = (colorId + 1) % CORE_COUNT_OF(palette);

        for (int i = 0; i < NSC; ++i)
        {
            auto v1 = qs * (qc * (K * pts[i * 2]) + cc) + cs;
            auto v2 = qs * (qc * (K * pts[i * 2 + 1]) + cc) + cs;

            mesh.addLine(v1, v2);
        }

        auto ppv = qs * (qc * (K.mulBy2dRight(cam.intrinsics.principal) * scale) + cc) + cs;

        mesh.addLine(ppv, qs * (qc * (K * center) + cc) + cs);
    }
}

void CalibrationHelpers::drawPly(Mesh3D &mesh, ObservationList &list)
{
    mesh.currentColor = RGBColor(~0u);
    for (auto& pt: list)
    {
        mesh.addPoint(pt.point);
    }
}

void CalibrationHelpers::drawPly(Mesh3D &mesh, Photostation &ps, ObservationList &list, double scale)
{
    drawPly(mesh, list);
    drawPly(mesh, ps, scale);
}
