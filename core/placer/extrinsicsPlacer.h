#ifndef EXTRINSICSPLACER_H
#define EXTRINSICSPLACER_H

#include "core/xml/generated/extrinsicsPlacerParameters.h"
#include "core/cameracalibration/calibrationLocation.h"
#include "core/function/function.h"

#include "core/camerafixture/fixtureScene.h"

namespace corecvs {

class SimplifiedScene
{
public:
    std::vector<CameraLocationData> cameras;
    std::vector<Vector3dd> points;

    /* This is not essential part of scene*/
    std::vector<int> idToScene;

    struct Observation {
        int cam;
        int point;
        Vector3dd obs;

        Observation(
                int cam,
                int point,
                Vector3dd obs) :
            cam(cam),
            point(point),
            obs(obs)
        {}

    };
    std::vector<Observation> obs;

    friend ostream& operator << (ostream &out, SimplifiedScene &toPrint)
    {

        out << "Cameras:" << toPrint.cameras.size() << endl;
        for (CameraLocationData &data : toPrint.cameras)
        {
            out << "   " << data << endl;
        }

        out << "Points" << toPrint.points.size() << endl;
        for (Vector3dd &data : toPrint.points)
        {
            out << "   " << data << endl;
        }

        cout << "Oservations: " << toPrint.obs.size() << endl;

        return out;
    }


    static SimplifiedScene extractSimpleScene (const FixtureScene *scene);
    static void            mergeSimpleScene   (FixtureScene *scene, const SimplifiedScene &simple);

};

struct SillyModel
{
    static const int CAM_MODEL_SIZE = Vector3dd::LENGTH + Quaternion::LENGTH;

    static int getInputNumber(const SimplifiedScene *scene)
    {
        return scene->cameras.size() * CAM_MODEL_SIZE + scene->points.size() * Vector3dd::LENGTH;
    }

    static int getOutputNumber(const SimplifiedScene *scene)
    {
        return scene->obs.size() * Vector3dd::LENGTH;
    }

    /* ====== */

    static int getCameraModelSize(ExtrinsicsPlacerParameters &params)
    {
        return  (params.lockOrientations() ? 0 : Quaternion::LENGTH) +
                (params.lockPositions()    ? 0 :  Vector3dd::LENGTH);
    }

    static int getPointsOffset(const SimplifiedScene *scene, ExtrinsicsPlacerParameters &params)
    {
        return 0;
    }

    static int getCameraOffset(const SimplifiedScene *scene, ExtrinsicsPlacerParameters &params)
    {
        return scene->points.size() * Vector3dd::LENGTH;
    }

    static int getInputNumber(const SimplifiedScene *scene, ExtrinsicsPlacerParameters &params)
    {
        return getPointsOffset(scene, params) + scene->points.size() * Vector3dd::LENGTH;
    }

    static int getOutputNumber(const SimplifiedScene *scene, ExtrinsicsPlacerParameters &params)
    {
        return scene->obs.size() * Vector3dd::LENGTH;
    }
};

class SillyNormalizer : public FunctionArgs, public SillyModel
{
public:    
    SimplifiedScene *scene = NULL;

    SillyNormalizer(SimplifiedScene *scene) :
        FunctionArgs(getInputNumber(scene), getInputNumber(scene)),
        scene(scene)
    {
    }

    virtual void operator()(const double in[], double out[]) override;

};

class SillyCost : public FunctionArgs, public SillyModel
{
public:
    SimplifiedScene *scene = NULL;

    SillyCost(SimplifiedScene *scene) :
        FunctionArgs(getInputNumber(scene), getOutputNumber(scene)),
        scene(scene)
    {
    }

    virtual void operator()(const double in[], double out[]) override;

    static vector<double>  sceneToModel  (const SimplifiedScene &scene);
    static void sceneFromModel(SimplifiedScene &scene, const vector<double> &model);
};



class SillyCostMask : public FunctionArgs, public SillyModel
{
public:
    SimplifiedScene *scene = NULL;
    ExtrinsicsPlacerParameters params;



    SillyCostMask(SimplifiedScene *scene, ExtrinsicsPlacerParameters &params) :
        FunctionArgs(getInputNumber(scene, params), getOutputNumber(scene, params)),
        scene(scene)
    {
    }

    virtual void operator()(const double in[], double out[]) override;

    vector<double>  sceneToModel  (const SimplifiedScene &scene);
    void sceneFromModel(SimplifiedScene &scene, const vector<double> &model);

};


class SillyNormalizerMask : public FunctionArgs, public SillyModel
{
public:
    SimplifiedScene *scene = NULL;
    ExtrinsicsPlacerParameters params;

    SillyNormalizerMask(SimplifiedScene *scene, ExtrinsicsPlacerParameters &params) :
        FunctionArgs(getInputNumber(scene, params), getInputNumber(scene, params)),
        scene(scene)
    {
    }

    virtual void operator()(const double in[], double out[]) override;

};


/** Most trival placer that optimises rays from observations by moving the cameras **/
class ExtrinsicsPlacer
{
public:
    ExtrinsicsPlacer();

    ExtrinsicsPlacerParameters params;

    void triangulate (FixtureScene *scene);
    void place (FixtureScene *scene);

    double getCost(FixtureScene *scene);
};


}

#endif // EXTRINSICSPLACER_H
