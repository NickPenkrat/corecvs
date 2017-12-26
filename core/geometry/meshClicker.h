#ifndef MESHCLICKER_H
#define MESHCLICKER_H

#include "core/geometry/mesh3d.h"

namespace corecvs {

/* Please check raytraceRenderer for some related code */

/*class ClickIntersection {

    double
};*/


/* This is a basic */
class MeshClicker
{
public:

    Mesh3D *targetMesh = NULL;
    CameraModel *model = NULL;

    MeshClicker();

    double vertexTolerance = 4;
    double   edgeTolerance = 4;

    enum GeometryFilter {
       VERTEX = 0x01,
       EDGE   = 0x02,
       FACE   = 0x04,
       ALL    = VERTEX | EDGE | FACE
    };



    virtual bool trace(const Ray3d &ray, GeometryFilter filter = GeometryFilter::ALL);
    virtual bool trace(const Vector2dd &ray, GeometryFilter filter = GeometryFilter::ALL);

    virtual bool vertexClicked( double &t, int vertexNum) { return false; }
    virtual bool edgeClicked  ( double &t, int edgeNum)   { return false; }
    virtual bool faceClicked  ( double &t, int vertexNum) { return false; }


};


} // namespace corecvs

#endif // MESHCLICKER_H
