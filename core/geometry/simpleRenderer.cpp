#include "simpleRenderer.h"
#include "mesh3d.h"
#include "rgb24Buffer.h"

namespace corecvs {

SimpleRenderer::SimpleRenderer() :
    backfaceClip(false),
    drawFaces(true),
    drawEdges(true),
    drawVertexes(true),
    zBuffer(NULL)
{
}

void SimpleRenderer::render(Mesh3D *mesh, RGB24Buffer *buffer)
{
    if (mesh == NULL || buffer == NULL)
    {
        return;
    }

    delete_safe(zBuffer);
    zBuffer = new AbstractBuffer<double>(buffer->getSize(), std::numeric_limits<double>::max());

    for(size_t f = 0; f < mesh->faces.size(); f++)
    {
        Vector3d32 face = mesh->faces[f];

        Triangle2dd triang;
        triang.p1() = (modelviewMatrix * mesh->vertexes[face[0]]).project();
        triang.p2() = (modelviewMatrix * mesh->vertexes[face[1]]).project();
        triang.p3() = (modelviewMatrix * mesh->vertexes[face[2]]).project();

        RGBColor color = RGBColor::Blue();
        if (mesh->hasColor) {
            color = mesh->facesColor[f];
        }
        TriangleSpanIterator it(triang);
        while (it.step()) {
            LineSpanInt span = it.getSpan();
            buffer->drawHLine(span.x1, span.cy, span.x2, color);
        }
    }

    for(size_t e = 0; e < mesh->edges.size(); e++)
    {
        Vector2d32 edge = mesh->edges[e];

        Vector2dd position1 = (modelviewMatrix * mesh->vertexes[edge[0]]).project();
        Vector2dd position2 = (modelviewMatrix * mesh->vertexes[edge[1]]).project();

        RGBColor color = RGBColor::Green();
        if (mesh->hasColor) {
            color = mesh->edgesColor[e];
        }
        buffer->drawLine(position1, position2, color);
    }

    for(size_t p = 0; p < mesh->vertexes.size(); p++)
    {
        Vector2dd position = (modelviewMatrix * mesh->vertexes[p]).project();
        cout << "SimpleRenderer::render(): " << mesh->vertexes[p] << " => " << position  << endl;

        RGBColor color = RGBColor::Red();
        if (mesh->hasColor) {
            color = mesh->vertexesColor[p];
        }
        buffer->drawPixel(position.x(), position.y(), color);
    }
}

void SimpleRenderer::fragmentShader(AttributedLineSpan &span)
{
    /**/
}

SimpleRenderer::~SimpleRenderer()
{
    delete_safe(zBuffer);
}




} // namespace corecvs
