#ifndef SELECTABLEGEOMETRYFEATURES_H
#define SELECTABLEGEOMETRYFEATURES_H

#include <algorithm>
#include <vector>
#include "vector2d.h"
#include "vector3d.h"


namespace corecvs {

using std::vector;

/* Move this to separate class */
struct PointObservation
{
    Vector3dd point;
    Vector2dd projection;

    PointObservation(
            Vector3dd _point      = Vector3dd(0),
            Vector2dd _projection = Vector2dd(0)
    ) : point(_point),
        projection(_projection)
    {}


    inline double &x()
    {
        return point.x();
    }

    inline double &y()
    {
        return point.y();
    }

    inline double &z()
    {
        return point.z();
    }

    inline double &u()
    {
        return projection.x();
    }

    inline double &v()
    {
        return projection.y();
    }
};


class ObservationList : public std::vector<PointObservation>
{
    public:

};


class SelectableGeometryFeatures
{
public:
    struct VertexPath;

    struct Vertex
    {
        bool        mSelected;
        Vector2dd   position;
        double      weight;
        VertexPath *ownerPath;

        explicit Vertex(const Vector2dd &_position = Vector2dd(0.0));

        bool isSelected();

        bool isInPath() {
            return (ownerPath != NULL);
        }
    };

    typedef Vector2dd * TargetPoint;

    /** Points **/
    vector<Vertex*> mPoints;
    vector<Vertex*> mSelectedPoints;

    Vertex *findClosest(const Vector2dd &position);
    //Vertex *lastVertex();

    struct VertexPath
    {
        bool mSelected;
        vector<Vertex *> vertexes;

        bool isSelected();
        bool isEmpty();
    };

    /** Pathes **/
    vector<VertexPath *> mPaths;
    vector<VertexPath *> mSelectedPaths;

    VertexPath *appendNewPath();
    void deletePath(VertexPath *path);

    void addVertexToPath(Vertex *vertex, VertexPath *path);
    void removeVertexFromPath(Vertex *vertex, bool purgeEmptyPath = true);

    Vertex* appendNewVertex(const Vector2dd &point = Vector2dd(0.0));
    void deleteVertex(const Vector2dd &point);
    void deleteVertex(Vertex *vertex);

    void clearAll();

    void addSelection   (VertexPath *path);
    void removeSelection(VertexPath *path);

    void addSelection   (Vertex *vertex);
    void removeSelection(Vertex *vertex);


    void deselectAllPoints();
    void deselectAllPath();
    void deselectAll();

    /****/
    bool hasSinglePointsSelected();

    SelectableGeometryFeatures();
    virtual ~SelectableGeometryFeatures();

    /* Helper function */
    vector<vector<Vector2dd> > getLines();
};

}

#endif // SELECTABLEGEOMETRYFEATURES_H
