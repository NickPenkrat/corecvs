#ifndef EXTRINSICSPLACER_H
#define EXTRINSICSPLACER_H

#include "core/cameracalibration/calibrationLocation.h"
#include "core/function/function.h"

#include "core/camerafixture/fixtureScene.h"

namespace corecvs {

class SimplifiedScene
{
public:
    std::vector<CameraLocationData> cameras;
    std::vector<Vector3dd> points;

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

    int getInputNumber() const
    {
        return cameras.size() * 7 + points.size() * 3;
    }

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

};


class SillyNormalizer : public FunctionArgs
{
public:
    SimplifiedScene *scene = NULL;

    SillyNormalizer(SimplifiedScene *scene) :
        FunctionArgs(scene->getInputNumber(), scene->getInputNumber()),
        scene(scene)
    {
    }

    virtual void operator()(const double in[], double out[]) override;

};

/* Ray angle is used as the base for cost function */
class SillyCost : public FunctionArgs
{
public:
    SimplifiedScene *scene = NULL;

    SillyCost(SimplifiedScene *scene) :
        FunctionArgs(scene->getInputNumber(), scene->obs.size() * 3),
        scene(scene)
    {
    }

    virtual void operator()(const double in[], double out[]) override;

    static vector<double>  sceneToModel  (const SimplifiedScene &scene);
    static void sceneFromModel(SimplifiedScene &scene, const vector<double> &model);

#if 0
    static void sceneToModel  (const FixtureScene *scene,       vector<double> &model);
    static void sceneFromModel(      FixtureScene *scene, const vector<double> &model);
#endif
};



/** Most trival placer that optimises rays from observations by moving the cameras **/
class ExtrinsicsPlacer
{
public:
    ExtrinsicsPlacer();

    void place (FixtureScene *scene);
};


}

#endif // EXTRINSICSPLACER_H
