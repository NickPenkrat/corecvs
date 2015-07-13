#ifndef SELECTABLEGEOMETRYFEATURES_H
#define SELECTABLEGEOMETRYFEATURES_H

#include <algorithm>
#include <vector>
#include "vector2d.h"

namespace corecvs {

using std::vector;

class SelectableGeometryFeatures
{
public:
    struct VertexPath;

    struct Vertex
    {
        bool        isSelected;
        Vector2dd   position;
        double      weight;
        VertexPath *ownerPath;

        explicit Vertex(const Vector2dd &_position);

        bool isInPath() {
            return (ownerPath != NULL);
        }
    };

    typedef Vector2dd * TargetPoint;

    /** Points **/
    vector<Vertex > mPoints;
    vector<Vertex*> mSelectedPoints;

    void addSelection(Vertex &vertex);
    void removeSelection(Vertex &vertex);
    void deselectAllPoints();


    Vertex *findClosest(const Vector2dd &position);
    //Vertex *lastVertex();

    struct VertexPath
    {
        bool isSelected;
        vector<Vertex *> vertexes;
    };

    /** Pathes **/
    vector<VertexPath> mPaths;
    vector<VertexPath *> mSelectedPaths;

    void appendPath();
    void deselectAllPath();
    void addVertexToPath(Vertex *vertex, VertexPath *path);
    void addSelection(VertexPath &path);

    void deleteVertex(const Vector2dd &point);
    void deleteVertex(Vertex *vertex);
    void deleteVertex(int num);

    void addVertex(const Vertex &vertex);
    void addVertex(const Vector2dd &point);

    void clearAll();
    void deselectAll();

    /****/
    bool hasSinglePointsSelected();

    SelectableGeometryFeatures();
    virtual ~SelectableGeometryFeatures();
};

}

#endif // SELECTABLEGEOMETRYFEATURES_H
