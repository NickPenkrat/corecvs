#ifndef SCENE_FEATURE_POINT_H
#define SCENE_FEATURE_POINT_H

#include <string>
#include <unordered_map>

#include "fixtureCamera.h"
#include "imageKeyPoints.h"
#include "wildcardablePointerPair.h"


/* Presentation related */
#include "rgbColor.h"

namespace corecvs
{
    template<typename U, typename V>
    class WildcardablePointerPair;
}


namespace corecvs {

class SceneFeaturePoint;


/**
 *
 *   Observation is a class that decribes the result of point beening visble from a particular camera
 *   As with other FixtureScene parts it's contence is governed by guidelines not strict riles.
 *
 *   Working algorithms using this structure actually describe it's own rules. This class is mostly a container
 *
 **/
class SceneObservation
{
public:
    SceneObservation(
              FixtureCamera     *cam = nullptr
            , SceneFeaturePoint *sfp = nullptr
            , Vector2dd          obs = Vector2dd::Zero()
            , CameraFixture     *fix = nullptr)
        : camera(cam)
        , cameraFixture(fix)
        , featurePoint(sfp)
        , observation(obs)
        , accuracy(0.0)
        , observDir(0.0)
        , onDistorted(false)
    {}

    /**
     *   Id of the camera that observese the point
     **/
    FixtureCamera      *camera;

    /**
     *   Id of the camera fixture that observese the point
     **/
    CameraFixture      *cameraFixture;

    /**
     *   Point that is observed
     **/
    SceneFeaturePoint  *featurePoint;
    Vector2dd           observation;
    Vector2dd           accuracy;
    Vector3dd           observDir;                  /**< Ray to point from camera origin - this is helpful when camera is not projective */
    bool                onDistorted;                /**< true when observation belongs to source-distorted image, def: we assume working with points on undistorted images */

  //MetaContainer       meta;                       /* not used */

    KeyPointArea    keyPointArea;

    double          &x() { return observation.x(); }
    double          &y() { return observation.y(); }

    std::string     getPointName();

    int             ensureDistorted(bool distorted = true);
    Vector2dd       getDistorted(bool distorted = true) const;

private:
    FixtureCamera  *getCameraById(FixtureCamera::IdType id);

public:
    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(observDir   , Vector3dd(0.0) , "observDir");
        visitor.visit(observation , Vector2dd(0.0) , "observation");
        visitor.visit(accuracy    , Vector2dd(0.0) , "accuracy");
        visitor.visit(onDistorted , false          , "onDistorted");

        keyPointArea.accept<VisitorType>(visitor);

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
        POINT_UNKNOWN               = 0x00,
        POINT_USER_DEFINED          = 0x01,  /**< Point that comes from a file */
        POINT_RECONSTRUCTED         = 0x02,
        POINT_TEMPORARY             = 0x04,
        POINT_TRIANGULATE           = 0x08,
        POINT_RAW_DETECTED          = 0x10,
        POINT_RAW_DETECTED_MATCHED  = 0x20,
        POINT_ALL                   = 0xFF
    };
    PointType                   type;

    static inline const char *getTypeName(const PointType &value)
    {
        switch (value)
        {
            case POINT_UNKNOWN       : return "POINT_UNKNOWN";
            case POINT_USER_DEFINED  : return "USER_DEFINED" ;
            case POINT_RECONSTRUCTED : return "RECONSTRUCTED";
            case POINT_TEMPORARY     : return "TEMPORARY"    ;
            case POINT_TRIANGULATE   : return "TRIANGULATE"  ;
            case POINT_RAW_DETECTED  : return "RAW_DETECTED" ;
            default                  : return "Not in range" ;
        }
    }

    void setPosition(const Vector3dd &position, PointType type = POINT_USER_DEFINED)
    {
        this->position = position;
        this->hasKnownPosition = true;
        this->type = type;
    }

    Vector3dd getDrawPosition(bool preferReprojected = false, bool forceKnown = false) const;

    /**
     * This method triangulates a point based on its observations
     *
     * \param use__ - should we use observations from observations or observations__
     * \param mask - to select observations to use
	 * \param numProjections - if non-null, returns number of projections
     *
     **/
    Vector3dd triangulate(bool use__ = false, std::vector<int> *mask = nullptr, uint* numProjections = nullptr);


    /** Observation related block */
    typedef std::unordered_map<FixtureCamera *, SceneObservation> ObservContainer;

    ObservContainer observations;

    std::unordered_map<WildcardablePointerPair<CameraFixture, FixtureCamera>, SceneObservation> observations__;

    /* This is a presentation related block we should move it to derived class */
    RGBColor        color;

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

    bool hasObservation(FixtureCamera *cam) { return getObservation(cam) != nullptr; }

    SceneObservation *getObservation(FixtureCamera *cam);

    void removeObservation(SceneObservation *);

    int  ensureDistortedObservations(bool distorted = true);    // convert to the needed type of all observations

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
            /* We don't load observations here*/

            /* We resort it to make compare easier. We should find a way to make it more stable */
            vector<FixtureCamera *> toSort;
            for (auto &it : observations) {
                toSort.push_back(it.first);
            }

            std::sort(toSort.begin(), toSort.end(), [](const FixtureCamera *first, const FixtureCamera *second) {
                return first->nameId < second->nameId;
            });

            int i = 0;
            for (auto &it : toSort)
            {
                SceneObservation &observ = observations[it];
                char buffer[100];
                snprintf2buf(buffer, "obsrv[%d]", i++);
                visitor.visit(observ, observ, buffer);
            }
        }
        else
        {
            observations.clear();
            SceneObservation observ0;
            for (int i = 0; i < observeSize; i++)
            {
                char buffer[100];
                snprintf2buf(buffer, "obsrv[%d]", i);
                SceneObservation observ;
                observ.featurePoint = this;         // we need to set it before visit()
                visitor.visit(observ, observ0, buffer);

                if (observ.camera == NULL)          // when we load the old format file with another field name
                {
                    snprintf2buf(buffer, "obsereve[%d]", i);
                    visitor.visit(observ, observ0, buffer);
                }

                observations[observ.camera] = observ;
                observations__[WPP(observ.cameraFixture, observ.camera)] = observ;
            }
        }
    }

};


class FixtureSceneGeometry : public FixtureScenePart, public FlatPolygon
{
public:
    FixtureSceneGeometry(FixtureScene * owner = NULL) :
        FixtureScenePart(owner)
    {}

    /** Related points container */
    typedef std::vector<SceneFeaturePoint *> RelatedPointsContainer;

    RelatedPointsContainer relatedPoints;


    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        FlatPolygon::accept(visitor);
    }

};

} // namespace corecvs

#endif // SCENE_FEATURE_POINT_H
