#ifndef RECONSTRUCTIONSTRUCTS
#define RECONSTRUCTIONSTRUCTS

#include <vector>
#include <string>

#include "vector2d.h"
#include "vector3d.h"
#include "imageKeyPoints.h"

struct PointProjection
{
    corecvs::Vector2dd projection;
    int photostationId;
    int cameraId;
    int featureId;    

    template<typename V>
    void accept(V &visitor)
    {
        visitor.visit(projection,     corecvs::Vector2dd(0.0, 0.0), "projection");
        visitor.visit(photostationId, 0,                            "photostationId");
        visitor.visit(cameraId,       0,                            "cameraId");
        visitor.visit(featureId,      -1,                           "featureId");
    }
};

// TODO: Fix underscored names
struct PointObservation__
{
    corecvs::Vector3dd worldPoint;

    std::vector<PointProjection> projections;

    bool updateable = true;

    template<typename V>
    void accept(V &visitor)
    {
        visitor.visit(worldPoint,  corecvs::Vector3dd(0.0, 0.0, 0.0), "worldPoint");
        visitor.visit(projections, "projections");
        visitor.visit(updateable,  true, "updateable");
    }
};
struct CameraObservation
{
    std::string sourceFileName;
    std::string undistortedFileName;
    std::string descriptorFileName;

    std::vector<KeyPoint> keyPoints;

    template<typename V>
    void accept(V &visitor)
    {
        visitor.visit(sourceFileName,      std::string(""), "sourceFileName");
        visitor.visit(undistortedFileName, std::string(""), "undistortedFileName");
        visitor.visit(descriptorFileName,  std::string(""), "descriptorFileName");

        visitor.visit(keyPoints, "keyPoints");
    }
};


#endif
