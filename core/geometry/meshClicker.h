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
    SpecifyTarget targetElements = MESH_ALL;

    MeshClicker();

    double vertexTolerance = 4;
    double   edgeTolerance = 4;


    virtual bool intersect(Ray3d &ray);
    
#if 0
    virtual bool vertexClicked( double &t, int &vertexNum);
    virtual bool edgeClicked  ( double &t, int &edgeNum);
    virtual bool faceClicked  ( double &t, int &vertexNum);
#endif

};


} // namespace corecvs

#endif // MESHCLICKER_H
