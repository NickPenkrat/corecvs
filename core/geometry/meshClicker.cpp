#include "core/geometry/meshClicker.h"

namespace corecvs {

MeshClicker::MeshClicker()
{
    
}

bool MeshClicker::intersect(Ray3d &ray)
{
    if (targetMesh == NULL)
        return false;

    for (int i = 0; i < targetMesh->vertexes.size(); i++)
    {
        
    }
    return false;
}

#if 0
bool MeshClicker::vertexClicked(double &t, int vertexNum)
{
}

bool MeshClicker::edgeClicked(double &t, int edgeNum)
{
    if (targetMesh == NULL)
        return false;

}

bool MeshClicker::faceClicked(double &t, int vertexNum)
{
    if (targetMesh == NULL)
        return false;

}
#endif


} // namespace corecvs

