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

namespace std {

template<typename U, typename V>
struct hash<corecvs::WildcardablePointerPair<U, V>>
{
    size_t operator() (const corecvs::WildcardablePointerPair<U, V> &wpp) const
    {
        return std::hash<U*>()(wpp.u) ^ (31 * std::hash<V*>()(wpp.v));
    }
};

} // namespace std


namespace corecvs {

class SceneFeaturePoint;

class SceneObservation
{
public:
    SceneObservation(FixtureCamera* cam = nullptr
            , SceneFeaturePoint *sfp = nullptr
            , Vector2dd          obs = Vector2dd(0)
            , CameraFixture     *fix = nullptr)
        : camera(cam)
        , cameraFixture(fix)
        , featurePoint(sfp)
        , observation(obs)
        , accuracy(0.0)
        , observDir(0.0)
        , isKnown(false)
    {}

    FixtureCamera      *camera;
    CameraFixture      *cameraFixture;
    SceneFeaturePoint  *featurePoint;
    Vector2dd           observation;
    Vector2dd           accuracy;
    Vector3dd           observDir;      /* Ray to point */
    bool                isKnown;
    MetaContainer       meta;
    
    double &x() { return observation.x(); }
    double &y() { return observation.y(); }

    std::string     getPointName();

private:
    FixtureCamera  *getCameraById(FixtureCamera::IdType id);

public:
    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(observDir   , Vector3dd(0.0) , "observDir");
        visitor.visit(observation , Vector2dd(0.0) , "observation");
        visitor.visit(accuracy    , Vector2dd(0.0) , "accuracy");
        visitor.visit(isKnown     , false          , "isKnown");

        FixtureCamera::IdType id = 0;
        if (camera != NULL) {
            id = camera->getObjectId();
        }
        visitor.visit(id, (uint64_t)0, "camera");

        if (visitor.isLoader())
        {
            camera = getCameraById(id);
            if (camera != NULL) {
                cameraFixture = camera->cameraFixture;
            }
        }
    }
};

template<typename U, typename V>
class WildcardablePointerPair
{
public:
#if !defined(WIN32) || (_MSC_VER >= 1900) // Sometime in future (when we switch to VS2015 due to https://msdn.microsoft.com/library/hh567368.apx ) we will get constexpr on windows
    static constexpr U* UWILDCARD = nullptr;
    static constexpr V* VWILDCARD = nullptr;
#else
    static U* const UWILDCARD;
    static V* const VWILDCARD;
#endif
    typedef U* UTYPE;
    typedef V* VTYPE;

    WildcardablePointerPair(U* u = UWILDCARD, V* v = VWILDCARD) : u(u), v(v)
    {
    }

    bool isWildcard() const
    {
        return u == UWILDCARD || v == VWILDCARD;
    }

    // Yes, this is NOT transitive (and you should use wildcarded wpps only for indexing not for insertion)
    bool operator== (const WildcardablePointerPair<U, V> &wpp) const
    {
        return (u == UWILDCARD || wpp.u == UWILDCARD || u == wpp.u) &&
               (v == VWILDCARD || wpp.v == VWILDCARD || v == wpp.v);
    }

    // This operator IS transitive
    bool operator< (const WildcardablePointerPair<U, V> &wpp) const
    {
        return u == wpp.u ? v < wpp.v : u < wpp.u;
    }

    U* u;
    V* v;
};
typedef WildcardablePointerPair<CameraFixture, FixtureCamera> WPP;


class SceneFeaturePoint : public FixtureScenePart
{
public:
    std::string                 name;

    /** This is a primary position of the FeaturePoint. The one that is know by direct measurement */
    Vector3dd                   position;
    bool                        hasKnownPosition;
    /*
     * Here we'll store some estimation for inverse of covariance matrix
     * So for delta v we'll get v'Av = Mahlanobis distance (which has
     * chi-squared distribution with 3 dof)
     */
    Matrix33                    accuracy;
    // Gets p-value using covariance estimation using one-sided test
    double                      queryPValue(const corecvs::Vector3dd &q) const;

    /** This is a position that is achived by reconstruction */
    Vector3dd                   reprojectedPosition;
    bool                        hasKnownReprojectedPosition;

    enum PointType {
        POINT_UNKNOWN       = 0x00,
        POINT_USER_DEFINED  = 0x01,  /**< Point that comes from a file */
        POINT_RECONSTRUCTED = 0x02,
        POINT_TEMPORARY     = 0x04,
        POINT_TRIANGULATE   = 0x05,

        POINT_ALL           = 0xFF
    };

    PointType type;

    static inline const char *getTypeName(const PointType &value)
    {
        switch (value)
        {
            case POINT_USER_DEFINED  : return "USER_DEFINED" ;
            case POINT_RECONSTRUCTED : return "RECONSTRUCTED";
            case POINT_TEMPORARY     : return "TEMPORARY"    ;
            case POINT_TRIANGULATE   : return "TRIANGULATE"  ;
            default                  : return "Not in range" ;
        }
    }

    void setPosition(const Vector3dd &position, PointType type = POINT_USER_DEFINED)
    {
        this->position = position;
        this->hasKnownPosition = true;
        this->type = type;
    }

    Vector3dd triangulate(bool use__ = false, uint64_t mask = ~uint64_t(0));


    /** Observation related block */
    typedef std::unordered_map<FixtureCamera *, SceneObservation> ObservContainer;
    ObservContainer observations;

    std::unordered_map<WildcardablePointerPair<CameraFixture, FixtureCamera>, SceneObservation> observations__;


/* This is a presentation related block we should move it to derived class */
    RGBColor color;
/**/

    SceneFeaturePoint(FixtureScene * owner = NULL) :
        FixtureScenePart(owner),
        position(0.0),
        hasKnownPosition(false),
        reprojectedPosition(0.0),
        hasKnownReprojectedPosition(false),
        type(POINT_UNKNOWN),
        color(RGBColor::White())
    {}

    SceneFeaturePoint(Vector3dd _position, const std::string &_name = std::string(), FixtureScene * owner = NULL) :
        FixtureScenePart(owner),
        name(_name),
        position(_position),
        hasKnownPosition(true),
        reprojectedPosition(0.0),
        hasKnownReprojectedPosition(false),
        type(POINT_UNKNOWN),
        color(RGBColor::White())
    {}

    void transform(const Matrix33 &matrix, const Vector3dd &translate)
    {
        position = matrix * position + translate;
    }

    bool hasObservation(FixtureCamera *cam);
    SceneObservation *getObservation(FixtureCamera *cam);

    void removeObservation(SceneObservation *);


    /* Let it be so far like this */
    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(name                       , std::string("")   , "name");
        visitor.visit(position                   , Vector3dd(0.0)    , "position");
        visitor.visit(hasKnownPosition           , false             , "hasKnownPosition");
        visitor.visit(reprojectedPosition        , Vector3dd(0.0)    , "reprojectedPosition");
        visitor.visit(hasKnownReprojectedPosition, false             , "hasKnownReprojectedPosition");
        visitor.visit(type                       , POINT_UNKNOWN     , "type");
        visitor.visit(color                      , RGBColor::Black() , "color");

        int observeSize = (int)observations.size();
        visitor.visit(observeSize, 0, "observations.size");

        if (!visitor.isLoader())
        {
            int i = 0;
            /* We don't load observations here*/
            for (auto &it : observations)
            {
                SceneObservation &observ = it.second;
                char buffer[100]; snprintf2buf(buffer, "obsrv[%d]", i);
                visitor.visit(observ, observ, buffer);
                i++;
            }
        }
        else
        {
            SceneObservation observ0;
            for (int i = 0; i < observeSize; i++)
            {
                char buffer[100]; snprintf2buf(buffer, "obsrv[%d]", i);
                SceneObservation observ;
                observ.featurePoint = this;         // we need to set it before visit()
                visitor.visit(observ, observ0, buffer);

                if (observ.camera == NULL)          // when we load the old format file with another field name
                {
                    snprintf2buf(buffer, "obsereve[%d]", i);
                    visitor.visit(observ, observ0, buffer);
                }

                observations[observ.camera] = observ;
            }
        }
    }

};

} // namespace corecvs

#endif // SCENE_FEATURE_POINT_H
