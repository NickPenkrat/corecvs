#ifndef FIXTURE_SCENE_H_
#define FIXTURE_SCENE_H_

#include "fixtureScenePart.h"
#include "fixtureCamera.h"
#include "cameraFixture.h"
#include "sceneFeaturePoint.h"

/* In future Scene would like to control memory management for child objects */
//#define SCENE_OWN_ALLOCATOR_DRAFT

namespace corecvs {

class CameraFixture;
class StatusTracker;

class FixtureSceneFactory {

public:
    typedef std::function<FixtureScene *()> FixtureSceneCreateFunctor;

    /**
     * Be careful... manipulating (writing) this stuff is not thread safe
     * We all know were a singleton can lead you.
     **/
    std::map<std::string, FixtureSceneCreateFunctor> creators;

private:
    static std::unique_ptr<FixtureSceneFactory> instance;

public:
    static const char* DEFAULT_NAME;

    static FixtureSceneFactory *getInstance();

    FixtureScene *sceneFactory(const std::string &name = DEFAULT_NAME);

    void print();
};

/**
 * Heap of Calibration related stuff
 **/
class FixtureScene
{
public:
    typedef FixtureCamera        CameraType;
    typedef CameraPrototype      CameraPrototypeType;
    typedef CameraFixture        FixtureType;
    typedef SceneFeaturePoint    PointType;
    typedef FixtureSceneGeometry GeometryType;


    FixtureScene();

    /**
     * This field encodes the "silent transform that happens when natural world corrdinate system changes to
     * image related. This covers but is not restritced to the transition between:
     *
     *   Z axis pointing to the sky and Z axis pointing to camera optical axis
     *
     * So far you can't change right handed system to left-handed. This cavity needs to be addressed later.
     *
     *  Camera  World   |  World   Camera
     *    Z       X     |    X        Z
     *    Y      -Z     |    Y       -X
     *    X      -Y     |    Z       -Y
     *
     *  This transform only happens when you use ::positionCameraInStation() method. Thoough we encourage you to do so.
     *
     **/
    Affine3DQ                     worldFrameToCameraFrame;

    std::string                   nameId;

    std::string                   relativeImageDataPath;

    StatusTracker *               processState = nullptr;

    /* This is for future, when all the heap/memory will be completed */
#ifdef SCENE_OWN_ALLOCATOR_DRAFT
    vector<FixtureScenePart *>    mOwnedObjects;
#endif

    /**
     *  Creates and fills the observations with points. It optionally simulates camera by rounding the projection to nearest pixel
     *
     **/
    void projectForward(SceneFeaturePoint::PointType mask, bool round = false);
    void triangulate   (SceneFeaturePoint * point);

    /**
     * Accessors. This need to be redone to invert constness and make objects const, not the arrays
     **/
    const vector<CameraPrototype *>&    cameraPrototypes() const  { return mCameraPrototypes; }
          vector<CameraPrototype *>&    cameraPrototypes()        { return mCameraPrototypes; }


    const vector<CameraFixture *>&      fixtures() const       { return mFixtures; }
          vector<CameraFixture *>&      fixtures()             { return mFixtures; }

    const vector<FixtureCamera *>&      orphanCameras() const  { return mOrphanCameras; }
          vector<FixtureCamera *>&      orphanCameras()        { return mOrphanCameras; }

    const vector<SceneFeaturePoint *>&  featurePoints() const  { return mSceneFeaturePoints; }
          vector<SceneFeaturePoint *>&  featurePoints()        { return mSceneFeaturePoints; }

    const vector<FixtureSceneGeometry *>&  geometries() const  { return mGeomtery; }
          vector<FixtureSceneGeometry *>&  geometries()        { return mGeomtery; }


protected:

    vector<CameraPrototype *>      mCameraPrototypes;
    vector<CameraFixture *>        mFixtures;
    vector<FixtureCamera *>        mOrphanCameras;
    vector<SceneFeaturePoint *>    mSceneFeaturePoints;
    vector<FixtureSceneGeometry *> mGeomtery;



    template<typename T>
    using umwpp = std::unordered_map<WPP, T>;
    template<typename T>
    using umwppv= umwpp<std::vector<T>>;

    template<typename T>
    void deleteFixtureCameraUMWPP(umwpp<T> &uwpp, FixtureCamera* fc)
    {
        WPP wpp(WPP::UWILDCARD, fc);
        bool contains = false;
        do
        {
            contains = false;
            for (auto it = uwpp.begin(); it != uwpp.end(); ++it)
            {
                if (wpp == it->first)
                {
                    uwpp.erase(it);
                    contains = true;
                    break;
                }
            }
        } while(contains);
    }
    template<typename T>
    void deleteFixtureCameraUMWPP(umwpp<umwpp<T>> &uwpp, FixtureCamera* fc)
    {
        WPP wpp(WPP::UWILDCARD, fc);
        for (auto& it: uwpp)
            deleteFixtureCameraUMWPP(it.second, fc);
    }
    template<typename T>
    void deleteCameraFixtureUMWPP(umwpp<T> &uwpp, CameraFixture* fc)
    {
        WPP wpp(fc, WPP::VWILDCARD);
        bool contains = false;
        do
        {
            contains = false;
            for (auto it = uwpp.begin(); it != uwpp.end(); ++it)
            {
                if (wpp == it->first)
                {
                    uwpp.erase(it);
                    contains = true;
                    break;
                }
            }
        } while(contains);
    }
    template<typename T>
    void deleteCameraFixtureUMWPP(umwpp<umwpp<T>> &uwpp, CameraFixture* fc)
    {
        WPP wpp(fc, WPP::VWILDCARD);
        for (auto& it: uwpp)
            deleteCameraFixtureUMWPP(it.second, fc);
    }
    template<typename T>
    void deletePairUMWPP(umwpp<T> &uwpp, CameraFixture* cf, FixtureCamera* fc)
    {
        uwpp.erase(uwpp.find(WPP(cf, fc)));
    }
    template<typename T>
    void deletePairUMWPP(umwpp<umwpp<T>> &uwpp, CameraFixture* cf, FixtureCamera* fc)
    {
        for (auto& it: uwpp)
            deletePairUMWPP(it.second, cf, fc);
    }
    template<typename T>
    void vectorErase(std::vector<T> &v, const T &t)
    {
        v.erase(std::remove(v.begin(), v.end(), t), v.end());
    }

    /* Methods that actually create objects */
    virtual CameraPrototype      *fabricateCameraPrototype();
    virtual FixtureCamera        *fabricateCamera();
    virtual CameraFixture        *fabricateCameraFixture();
    virtual SceneFeaturePoint    *fabricateFeaturePoint();
    virtual FixtureSceneGeometry *fabricateSceneGeometry();

public:

    /**
     * Manipulation with structures
     **/
    virtual CameraPrototype      *createCameraPrototype();
    virtual FixtureCamera        *createCamera();
    virtual CameraFixture        *createCameraFixture();
    virtual SceneFeaturePoint    *createFeaturePoint();
    virtual FixtureSceneGeometry *createSceneGeometry();

    virtual void destroyObject    (FixtureScenePart *condemned);

    /* These methods completely purge objects from scene */
    virtual void deleteCamera         (FixtureCamera *camera);
    virtual void deleteCameraPrototype(CameraPrototype *cameraPrototype);
    virtual void deleteCameraFixture  (CameraFixture *fixture, bool recursive = true);
    virtual void deleteFixturePair    (CameraFixture *fixture, FixtureCamera *camera);
    virtual void deleteFeaturePoint   (SceneFeaturePoint *point);
    virtual void deleteSceneGeometry  (FixtureSceneGeometry *geometries);

    virtual void clear();

    /**
     *    Helper method to check data structure integrity
     *
     *    The restritions in this method are strongly recomended but not enforced
     *
     **/
    virtual bool checkIntegrity();

    /**
     *    This function takes top to bottom graph and updates all the backlinks to point correctly
     **/
    virtual bool integrityRelink();

    /**
     *  ATTENTION! METHOD COULD BE UNFINISHED
     *  This method merges another FixtureScene in this one with deep copy. This could also be used to clone.
     *  So far fixtures are not merged.
     **/
    virtual void merge(FixtureScene *other);

    virtual void positionCameraInFixture(CameraFixture *station, FixtureCamera *camera, const Affine3DQ &location);
    virtual void addCameraToFixture     (FixtureCamera *cam, CameraFixture *fixture);

    /**/
    virtual int getObeservationNumber(CameraFixture *fixture);
    virtual int getObeservationNumber(FixtureCamera *cam);


    /* Some debugger helpers */
    virtual void dumpInfo(ostream &out = std::cout, bool brief = false);

    // Transforms the whole world using scale*(QX+T) (FixtureCamera's inside CameraFixtures are not moved)
    virtual void transform(const corecvs::Affine3DQ &transformation, const double scale = 1.0);


    size_t totalObservations() const
    {
        size_t toReturn = 0;
        for (auto point : mSceneFeaturePoints) { toReturn += point->observations.size(); }
        return toReturn;
    }

    /**
     *
     **/
    void setPrototypeCount   (size_t count);
    void setFixtureCount     (size_t count);
    void setOrphanCameraCount(size_t count);
    void setFeaturePointCount(size_t count);
    void setGeometryCount    (size_t count);


    /* This performs full search. It should not */
    FixtureCamera     *getCameraById (FixtureScenePart::IdType id);
    CameraFixture     *getFixtureById(FixtureScenePart::IdType id);
    SceneFeaturePoint *getPointById  (FixtureScenePart::IdType id);

    SceneFeaturePoint *getPointByName(const std::string &name);


    template<class VisitorType, class SceneType = FixtureScene>
    void accept(VisitorType &visitor,
                bool loadCameras = true,
                bool loadFixtures = true,
                bool loadPoints = true,
                bool loadPrototypes = true,
                bool loadGeometry = true)
    {
        visitor.visit(relativeImageDataPath, std::string(""), "relativeImageDataPath");

        typedef typename SceneType::CameraPrototypeType   RealPrototypeType;
        typedef typename SceneType::CameraType            RealCameraType;
        typedef typename SceneType::FixtureType           RealFixtureType;
        typedef typename SceneType::PointType             RealPointType;
        typedef typename SceneType::GeometryType          RealGeometryType;

        if (loadPrototypes)
        {
            int oprotoSize = (int)mCameraPrototypes.size();
            visitor.visit(oprotoSize, 0, "cameraprototypes.size");

            setPrototypeCount(oprotoSize);

            for (size_t i = 0; i < (size_t)oprotoSize; i++)
            {
                char buffer[100];
                snprintf2buf(buffer, "cameraprototypes[%d]", i);
                visitor.visit(*static_cast<RealPrototypeType *>(mCameraPrototypes[i]), buffer);
            }
        }

        /* So far compatibilty is on */
        /* Orphan cameras */
        if (loadCameras)
        {
            int ocamSize = (int)mOrphanCameras.size();
            visitor.visit(ocamSize, 0, "orphancameras.size");

            setOrphanCameraCount(ocamSize);

            for (size_t i = 0; i < (size_t)ocamSize; i++)
            {
                char buffer[100];
                snprintf2buf(buffer, "orphancameras[%d]", i);
                visitor.visit(*static_cast<RealCameraType *>(mOrphanCameras[i]), buffer);
            }
        }

        /* Fixtures*/
        if (loadFixtures)
        {
            int stationSize = (int)mFixtures.size();
            visitor.visit(stationSize, 0, "stations.size");

            setFixtureCount(stationSize);

            for (size_t i = 0; i < (size_t)stationSize; i++)
            {
                char buffer[100];
                snprintf2buf(buffer, "stations[%d]", i);
                visitor.visit(*static_cast<RealFixtureType *>(mFixtures[i]), buffer);
            }
        }

        /* Points */
        if (loadPoints)
        {
            int pointsSize = (int)mSceneFeaturePoints.size();
            visitor.visit(pointsSize, 0, "points.size");

            setFeaturePointCount(pointsSize);

            for (size_t i = 0; i < (size_t)pointsSize; i++)
            {
                char buffer[100];
                snprintf2buf(buffer, "points[%d]", i);
                visitor.visit(*static_cast<RealPointType *>(mSceneFeaturePoints[i]), buffer);
            }
        }

        if (loadGeometry)
        {
            int geometrySize = (int)mGeomtery.size();
            visitor.visit(geometrySize, 0, "geometry.size");

            setGeometryCount(geometrySize);

            for (size_t i = 0; i < (size_t)geometrySize; i++)
            {
                char buffer[100];
                snprintf2buf(buffer, "geometry[%d]", i);
                visitor.visit(*static_cast<RealGeometryType *>(mGeomtery[i]), buffer);
            }
        }

    }

    virtual ~FixtureScene();
};

} // namespace corecvs


#endif // FIXTURE_SCENE_H_
