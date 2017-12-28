#ifndef MESHCLICKER_H
#define MESHCLICKER_H

#include "core/geometry/mesh3d.h"

namespace corecvs {

/* Please check raytraceRenderer for some related code */

/*class ClickIntersection {

    double
};*/

/** Base clicker for undecorated mesh **/
class MeshClicker
{
public:

    enum SpecifyTarget {
        MESH_EDGES    = 0x1,
        MESH_VERTEXES = 0x2,
        MESH_FACES    = 0x4,
        MESH_ALL      = MESH_EDGES | MESH_FACES | MESH_VERTEXES
    };


    Mesh3D *targetMesh = NULL;

    CameraModel *projection = NULL;
    SpecifyTarget targetElements = MESH_ALL;

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

    virtual bool vertexClicked(const double &t, int vertexNum);
    virtual bool edgeClicked  (const double &t, int edgeNum)  ;
    virtual bool faceClicked  (const double &t, int vertexNum);


    virtual bool intersect(Ray3d &ray);
    

};


} // namespace corecvs

#endif // MESHCLICKER_H
