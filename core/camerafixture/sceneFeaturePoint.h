#ifndef SCENE_FEATURE_POINT_H
#define SCENE_FEATURE_POINT_H

#include <string>
#include <unordered_map>

#include "fixtureCamera.h"


/* Presentation related */
#include "rgbColor.h"

namespace corecvs {

class SceneFeaturePoint;

class SceneObservation {
public:
    SceneObservation() {}

    FixtureCamera *     camera;
    SceneFeaturePoint * featurePoint;

    Vector2dd                 observation;
    Vector2dd                 accuracy;
    bool                      isKnown;
    MetaContainer             meta;

    /* Ray to point */
    Vector3dd                 observDir;
};


class SceneFeaturePoint : public FixtureScenePart
{
public:
    std::string name;

    /** This is a primary position of the FeaturePoint. The one that is know by direct measurement */
    Vector3dd position;
    bool hasKnownPosition;

    /** This is a position that is achived by reconstruction */
    Vector3dd reprojectedPosition;
    bool hasKnownReprojectedPosition;

    enum PointType{
        POINT_USER_DEFINED  = 0x01,  /**< Point that comes from a file */
        POINT_RECONSTRUCTED = 0x02,
        POINT_TEMPORARY     = 0x04,

        POINT_ALL           = 0xFF
    };

    PointType type;

    /** Observation related block */
    typedef std::unordered_map<FixtureCamera *, SceneObservation> ObservContainer;
    ObservContainer observations;




/* This is a presentation related block we should move it to derived class */
    RGBColor color;
/**/

    SceneFeaturePoint(FixtureScene * owner = NULL) :
        FixtureScenePart(owner),
        hasKnownPosition(false)
    {}


    SceneFeaturePoint(Vector3dd _position, const std::string &_name = std::string(), FixtureScene * owner = NULL) :
        FixtureScenePart(owner),
        name(_name),
        position(_position),
        hasKnownPosition(true)
    {}

    void transform(const Matrix33 &matrix, const Vector3dd &translate)
    {
        position = matrix * position + translate;
    }


};

}

#endif // SCENE_FEATURE_POINT_H
