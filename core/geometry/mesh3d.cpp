/**
 * \file mesh3d.cpp
 *
 * \date Dec 13, 2012
 **/
#include "mathUtils.h"      // M_PI
#include "mesh3d.h"

namespace corecvs {

void Mesh3D::addAOB(Vector3dd c1, Vector3dd c2)
{
    int vectorIndex = (int)vertexes.size();
    Vector3d32 startId(vectorIndex, vectorIndex, vectorIndex);

    vertexes.push_back(Vector3dd(c1.x(), c1.y(), c1.z()));
    vertexes.push_back(Vector3dd(c2.x(), c1.y(), c1.z()));
    vertexes.push_back(Vector3dd(c2.x(), c2.y(), c1.z()));
    vertexes.push_back(Vector3dd(c1.x(), c2.y(), c1.z()));

    vertexes.push_back(Vector3dd(c1.x(), c1.y(), c2.z()));
    vertexes.push_back(Vector3dd(c2.x(), c1.y(), c2.z()));
    vertexes.push_back(Vector3dd(c2.x(), c2.y(), c2.z()));
    vertexes.push_back(Vector3dd(c1.x(), c2.y(), c2.z()));

    faces.push_back(startId + Vector3d32(0, 1, 2));
    faces.push_back(startId + Vector3d32(0, 1, 2));
    faces.push_back(startId + Vector3d32(2, 3, 0));
    faces.push_back(startId + Vector3d32(7, 6, 5));
    faces.push_back(startId + Vector3d32(5, 4, 7));
    faces.push_back(startId + Vector3d32(0, 4, 5));
    faces.push_back(startId + Vector3d32(5, 1, 0));
    faces.push_back(startId + Vector3d32(1, 5, 6));
    faces.push_back(startId + Vector3d32(6, 2, 1));
    faces.push_back(startId + Vector3d32(2, 6, 7));
    faces.push_back(startId + Vector3d32(7, 3, 2));
    faces.push_back(startId + Vector3d32(3, 7, 4));
    faces.push_back(startId + Vector3d32(4, 0, 3));

    textureCoords.push_back(Vector2dd(0.0,0.0));
    textureCoords.push_back(Vector2dd(1.0,0.0));
    textureCoords.push_back(Vector2dd(1.0,1.0));
    textureCoords.push_back(Vector2dd(0.0,1.0));

    textureCoords.push_back(Vector2dd(0.0,0.0));
    textureCoords.push_back(Vector2dd(1.0,0.0));
    textureCoords.push_back(Vector2dd(1.0,1.0));
    textureCoords.push_back(Vector2dd(0.0,1.0));
}

void Mesh3D::addAOB(const AxisAlignedBoxParameters &box)
{
    Vector3dd measure(box.width(), box.height(), box.depth());
    Vector3dd center (box.x()    , box.y()     , box.z    ());


    addAOB(center - measure / 2.0, center + measure / 2.0);
}

void Mesh3D::addAOB(const AxisAlignedBox3d &box)
{
    addAOB(box.low(), box.high());
}

int Mesh3D::addPoint(Vector3dd point)
{
     vertexes.push_back(point);
     return vertexes.size() - 1;
}

void Mesh3D::addLine(Vector3dd point1, Vector3dd point2)
{
    int vectorIndex = (int)vertexes.size();
    Vector2d32 startId(vectorIndex, vectorIndex);

    vertexes.push_back(point1);
    vertexes.push_back(point2);

    edges.push_back(startId + Vector2d32(0, 1));

}

void Mesh3D::addSphere(Vector3dd center, double radius, int step)
{
    int vectorIndex = (int)vertexes.size();
    Vector3d32 startId(vectorIndex, vectorIndex, vectorIndex);

    double dphy =     M_PI / (step + 1);
    double dpsi = 2 * M_PI / (step + 1);

    for (int i = 0; i < step; i++)
    {
        for (int j = 0; j < step; j++)
        {
            double phi = dphy * i;
            double psi = dpsi * j;

            double x = radius * sin(phi) * sin(psi);
            double y = radius * sin(phi) * cos(psi);
            double z = radius * cos(phi);
            vertexes.push_back(center + Vector3dd(x,y,z));
        }
    }
}

void Mesh3D::addCamera(const CameraIntrinsics &cam, double len)
{
    //double aspect = cam.

    double farPlane  = len;
    double alphaH = cam.getHFov();
    double alphaV = cam.getVFov();

    double x, y;
    x = farPlane  * tan (alphaH);
    y = farPlane  * tan (alphaV);

    int vectorIndex = (int)vertexes.size();
    Vector2d32 startId(vectorIndex, vectorIndex);

    vertexes.push_back(Vector3dd(0,0,0));
    vertexes.push_back(Vector3dd( x, y, farPlane));
    vertexes.push_back(Vector3dd( x,-y, farPlane));
    vertexes.push_back(Vector3dd(-x,-y, farPlane));
    vertexes.push_back(Vector3dd(-x, y, farPlane));

    edges.push_back(Vector2d32(0,1) + startId);
    edges.push_back(Vector2d32(0,2) + startId);
    edges.push_back(Vector2d32(0,3) + startId);
    edges.push_back(Vector2d32(0,4) + startId);

    edges.push_back(Vector2d32(1,2) + startId);
    edges.push_back(Vector2d32(2,3) + startId);
    edges.push_back(Vector2d32(3,4) + startId);
    edges.push_back(Vector2d32(4,1) + startId);

    //SYNC_PRINT(("This 0x%X. Edges %d", this, edges.size()));
    //for (unsigned i = 0; i < edges.size(); i++)
    //{
    //    SYNC_PRINT(("Edges %d - [%d - %d]\n", i, edges[i].x(), edges[i].y()));
    //}

}

#if 0
void Mesh3D::addTruncatedCone(double r1, double r2, double length, int steps)
{
    int vectorIndex = vertexes.size();
    Vector3d32 startId(vectorIndex, vectorIndex, vectorIndex);

    /*for (int i = 0; i < steps; i++)
    {
        double angle1 = (i / (double)steps * 2.0 * M_PI);
        double angle2 = (((i + 1) % steps) / (double)steps * 2.0 * M_PI);

        double sa1 = sin(angle1);
        double ca1 = cos(angle1);

        double sa2 = sin(angle2);
        double ca2 = cos(angle2);

        glVertex3d(r1 * sa1, r1 * ca1 ,-length);
        glVertex3d(r1 * sa2, r1 * ca2 ,-length);

        glVertex3d(r2 * sa1, r2 * ca1 ,-0.0);
        glVertex3d(r2 * sa2, r2 * ca2 ,-0.0);

        glVertex3d(r1 * sa1, r1 * ca1 ,-length);
        glVertex3d(r2 * sa1, r2 * ca1 ,-0.0);
    }*/
}
#endif

void Mesh3D::dumpPLY(ostream &out)
{
    out << "ply" << std::endl;
    out << "format ascii 1.0" << std::endl;
    out << "comment made by ViMouse software" << std::endl;
    out << "comment This file is a saved stereo-reconstruction" << std::endl;
    out << "element vertex " << vertexes.size() << std::endl;
    out << "property float x" << std::endl;
    out << "property float y" << std::endl;
    out << "property float z" << std::endl;
    out << "property uchar red" << std::endl;
    out << "property uchar green" << std::endl;
    out << "property uchar blue" << std::endl;
    out << "element face " << faces.size() << std::endl;
    out << "property list uchar int vertex_index" << std::endl;
    out << "element edge " << edges.size() << std::endl;
    out << "property int vertex1" << std::endl;
    out << "property int vertex2" << std::endl;
    out << "end_header" << std::endl;

    for (unsigned i = 0; i < vertexes.size(); i++)
    {
        out << vertexes[i].x() << " "
            << vertexes[i].y() << " "
            << vertexes[i].z() << " "
            << (unsigned)(128) << " "
            << (unsigned)(128) << " "
            << (unsigned)(128) << std::endl;
    }

    for (unsigned i = 0; i < faces.size(); i++)
    {
        out << "3 "
            << faces[i].x() << " "
            << faces[i].y() << " "
            << faces[i].z() << " " << std::endl;
    }

    for (unsigned i = 0; i < edges.size(); i++)
    {
        out << edges[i].x() << " "
            << edges[i].y() << " "  << std::endl;
    }

//    SYNC_PRINT(("This 0x%X. Edges %d", this, edges.size()));

}

void Mesh3D::transform(const Matrix44 &matrix)
{
    for (unsigned i = 0; i < vertexes.size(); i++)
    {
        vertexes[i] = matrix * vertexes[i];
    }
}

Mesh3D Mesh3D::transformed(const Matrix44 &matrix)
{
    Mesh3D toReturn;
    toReturn = *this;
    toReturn.transform(matrix);
    return toReturn;
}

void Mesh3D::add(const Mesh3D &other)
{
    int newZero = vertexes.size();
    vertexes.reserve(vertexes.size() + other.vertexes.size());
    faces.reserve(faces.size() + other.faces.size());
    edges.reserve(edges.size() + other.edges.size());

    for (unsigned i = 0; i < other.vertexes.size(); i++)
    {
        vertexes.push_back(other.vertexes[i]);
    }

    for (unsigned i = 0; i < other.faces.size(); i++)
    {
        faces.push_back(other.faces[i] + Vector3d32(newZero, newZero, newZero));
    }

    for (unsigned i = 0; i < other.edges.size(); i++)
    {
        edges.push_back(other.edges[i] + Vector2d32(newZero, newZero));
    }
}



} /* namespace corecvs */
