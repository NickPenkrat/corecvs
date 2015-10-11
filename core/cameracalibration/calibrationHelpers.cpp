#include "calibrationHelpers.h"
#include "mesh3d.h"
#include "calibrationCamera.h"
#include "calibrationPhotostation.h"
#include "calibrationScene.h"
#include "selectableGeometryFeatures.h"
#include "abstractPainter.h"

using namespace corecvs;

RGBColor CalibrationHelpers::palette[] =
{
    RGBColor(0x762a83u),
    RGBColor(0xaf8dc3u),
    RGBColor(0xe7d4e8u),
    RGBColor(0xd9f0d3u),
    RGBColor(0x7fbf7bu),
    RGBColor(0x1b7837u)
};

void CalibrationHelpers::drawCamera(Mesh3D &mesh, CameraModel &cam, double scale)
{
    double w = cam.intrinsics.w();
    double h = cam.intrinsics.h();

    Vector3dd
            center      = Vector3dd( 0,  0,  0),
            center2     = Vector3dd( 0,  0,  1) * scale,
            topLeft     = Vector3dd( 0,  0,  1) * scale,
            topRight    = Vector3dd( w,  0,  1) * scale,
            bottomRight = Vector3dd( w,  h,  1) * scale,
            bottomLeft  = Vector3dd( 0,  h,  1) * scale;

    Vector3dd edges[] =
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

    const int edgenumber = CORE_COUNT_OF(edges) / 2;

    Vector3dd cc    = cam.extrinsics.position;
    Quaternion qc   = cam.extrinsics.orientation.conjugated();
    Matrix33 invK   = cam.intrinsics.getInvKMatrix33();


    for (int i = 0; i < edgenumber; ++i)
    {
        Vector3dd v1 = qc * (invK * edges[i * 2    ]) + cc;
        Vector3dd v2 = qc * (invK * edges[i * 2 + 1]) + cc;

        mesh.addLine(v1, v2);
    }

    Vector3dd ppv = qc * (invK.mulBy2dRight(cam.intrinsics.principal) * scale) + cc;

    mesh.addLine(ppv, qc * (invK * center) + cc);
}


void CalibrationHelpers::drawPly(Mesh3D &mesh, Photostation &ps, double scale)
{
    // Colorblind-safe palette
    CameraLocationData loc = ps.getLocation();
    Vector3dd  cs = loc.position;
    Quaternion qs = loc.orientation.conjugated();
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

    if (printNames)
    {
        AbstractPainter<Mesh3D> p(&mesh);
        mesh.mulTransform(Matrix44::Shift(ps.location.shift));
        mesh.setColor(RGBColor::Blue());
        p.drawFormatVector(scale / 5.0, scale / 5.0, 0, scale / 3.0, "TEST STRING", ps.name.c_str());
        mesh.popTransform();
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

void CalibrationHelpers::drawPly(Mesh3D &mesh, CalibrationFeaturePoint &fp, double scale)
{
    if (!largePoints) {
        mesh.addPoint(fp.position);
    } else {
        mesh.addIcoSphere(fp.position, scale / 100.0, 2);
    }

    if (printNames) {
        AbstractPainter<Mesh3D> p(&mesh);
        mesh.mulTransform(Matrix44::Shift(fp.position));
        mesh.setColor(RGBColor::Blue());
        p.drawFormatVector(scale / 5.0, scale / 5.0, 0, scale / 3.0, "%s", fp.name.c_str());
        mesh.popTransform();
    }
}

void CalibrationHelpers::drawScene(Mesh3D &mesh, CalibrationScene &scene, double scale)
{
    for (Photostation &ps: scene.stations)
    {
        drawPly(mesh, ps, scale);
    }

    for (CalibrationFeaturePoint &fp: scene.points)
    {
        drawPly(mesh, fp, scale);
    }
}


