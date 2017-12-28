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


bool MeshClicker::trace(const Ray3d &ray, GeometryFilter filter)
{
    if (targetMesh == NULL || projection == NULL) {
        return false;
    }

}

bool MeshClicker::trace(const Vector2dd &point, MeshClicker::GeometryFilter filter)
{
    if (targetMesh == NULL || projection == NULL) {
        return false;
    }

    if (filter & GeometryFilter::VERTEX)
    {
        for (size_t v = 0; v < targetMesh->vertexes.size(); v++)
        {
            Vector3dd &p = targetMesh->vertexes[v];
            if (!projection->isVisible(p)) {
                continue;
            }
            Vector2dd prj = projection->project(p);

            if ((point - prj).l2Metric() > vertexTolerance)
                continue;

            vertexClicked(0, v);
        }

    }

    if (filter & GeometryFilter::EDGE)
    {

    }

    if (filter & GeometryFilter::FACE)
    {
        //Ray3d ray = projection->reverse(point);

    }


}

bool MeshClicker::vertexClicked(const double &t, int vertexNum)
{
    return false;
}

bool MeshClicker::edgeClicked(const double &t, int edgeNum)
{
    return false;
}

bool MeshClicker::faceClicked(const double &t, int vertexNum)
{
    return false;
}


} // namespace corecvs

