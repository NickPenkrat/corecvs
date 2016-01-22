#ifndef SCENE_FEATURE_POINT_H
#define SCENE_FEATURE_POINT_H

#include <string>
#include <unordered_map>

#include "fixtureCamera.h"


/* Presentation related */
#include "rgbColor.h"

namespace corecvs
{
    template<typename U, typename V>
    class WildcardablePointerPair;
}

namespace std
{
template<typename U, typename V>
struct hash<corecvs::WildcardablePointerPair<U, V>>
{
    size_t operator() (const corecvs::WildcardablePointerPair<U, V> &wpp) const
    {
        return std::hash<U*>()(wpp.u) ^ (31 * std::hash<V*>()(wpp.v));
    }
};
}


namespace corecvs {

class SceneFeaturePoint;

class SceneObservation {
public:
    SceneObservation() {}

    FixtureCamera *     camera;
    CameraFixture *     cameraFixture;
    SceneFeaturePoint * featurePoint;

    Vector2dd                 observation;
    Vector2dd                 accuracy;
    bool                      isKnown;
    MetaContainer             meta;

    /* Ray to point */
    Vector3dd                 observDir;
};

template<typename U, typename V>
class WildcardablePointerPair
{
public:
    static constexpr U* UWILDCARD = nullptr;
    static constexpr V* VWILDCARD = nullptr;

    WildcardablePointerPair(U* u = UWILDCARD, V* v = VWILDCARD) : u(u), v(v)
    {
    }

    // Yes, this is not transitive (and you should use wildcarded wpps only for indexing not for insertion)
    bool operator== (const WildcardablePointerPair<U, V> &wpp) const
    {
        return (u == UWILDCARD || wpp.u == UWILDCARD || u == wpp.u) &&
               (v == VWILDCARD || wpp.v == VWILDCARD || v == wpp.v);
    }

    U* u;
    V* v;
};
typedef WildcardablePointerPair<CameraFixture, FixtureCamera> WPP;


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
    std::unordered_map<WildcardablePointerPair<CameraFixture, FixtureCamera>, SceneObservation> observations__;


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
