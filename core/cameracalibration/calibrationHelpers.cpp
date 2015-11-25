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

void CalibrationHelpers::drawCamera(Mesh3D &mesh, const CameraModel &cam, double scale)
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
    AbstractPainter<Mesh3D> p(&mesh);
    mesh.mulTransform(Matrix44::Shift(cc));
    p.drawFormatVector(0.0, 0.0, 0, scale / 50.0, "Cam: %s", cam.nameId.c_str());
    mesh.popTransform();
}


void CalibrationHelpers::drawPly(Mesh3D &mesh, const Photostation &ps, double scale)
{
    int colorId = 0;
    for (uint cam = 0; cam < ps.cameras.size(); ++cam)
    {
        mesh.currentColor = palette[colorId = (colorId + 1) % CORE_COUNT_OF(palette)];
        drawCamera(mesh, ps.getRawCamera(cam), scale);
    }
    corecvs::Vector3dd xps(scale,   0.0,   0.0),
                       yps(  0.0, scale,   0.0),
                       zps(  0.0,   0.0, scale),
                       ori(  0.0,   0.0,   0.0);
    mesh.currentColor = corecvs::RGBColor(255,   0,   0);
    mesh.addLine(ps.location * ori, ps.location * xps);
    mesh.currentColor = corecvs::RGBColor(  0, 255,   0);
    mesh.addLine(ps.location * ori, ps.location * yps);
    mesh.currentColor = corecvs::RGBColor(  0,   0, 255);
    mesh.addLine(ps.location * ori, ps.location * zps);

    if (printNames)
    {
        AbstractPainter<Mesh3D> p(&mesh);
        mesh.mulTransform(Matrix44::Shift(ps.location.shift));
        mesh.setColor(RGBColor::Blue());
        p.drawFormatVector(scale / 5.0, scale / 5.0, 0, scale / 3.0, "TEST STRING", ps.name.c_str());
        mesh.popTransform();
    }
}

void CalibrationHelpers::drawPly(Mesh3D &mesh, const ObservationList &list)
{
    mesh.currentColor = RGBColor(~0u);
    for (auto& pt: list)
    {
        mesh.addPoint(pt.point);
    }
}

void CalibrationHelpers::drawPly(Mesh3D &mesh, const Photostation &ps, const ObservationList &list, double scale)
{
    drawPly(mesh, list);
    drawPly(mesh, ps, scale);
}

void CalibrationHelpers::drawPly(Mesh3D &mesh, const CalibrationFeaturePoint &fp, double scale)
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

void CalibrationHelpers::drawScene(Mesh3D &mesh, const CalibrationScene &scene, double scale)
{
    for (Photostation *ps: scene.stations)
    {
        drawPly(mesh, *ps, scale);
    }

    for (CalibrationFeaturePoint *fp: scene.points)
    {
        drawPly(mesh, *fp, scale);
    }
}


