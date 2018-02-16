#include <limits>

#include "extrinsicsPlacer.h"
#include "core/camerafixture/cameraFixture.h"
#include "core/camerafixture/fixtureCamera.h"
#include "core/math/levenmarq.h"


namespace corecvs {

ExtrinsicsPlacer::ExtrinsicsPlacer()
{

}


void SillyNormalizer::operator ()(const double in[], double out[])
{
    if (scene == NULL) {
        return;
    }

    memcpy(out, in, sizeof(double) * outputs);

    for (size_t camId = 0; camId < scene->cameras.size(); camId++)
    {
        Quaternion rotor;
        rotor.loadFrom(&out[CAM_MODEL_SIZE * camId + 3]);
        rotor.normalise();
        rotor.storeTo (&out[CAM_MODEL_SIZE * camId + 3]);
    }
}

void SillyCost::operator ()(const double in[], double out[])
{
    if (scene == NULL) {
        return;
    }

    for (size_t obsId = 0; obsId < scene->obs.size(); obsId++)
    {
        SimplifiedScene::Observation &obs = scene->obs[obsId];

        Affine3DQ cam;
        cam.shift.loadFrom(& in[CAM_MODEL_SIZE * obs.cam]);
        cam.rotor.loadFrom(& in[CAM_MODEL_SIZE * obs.cam + Vector3dd::LENGTH]);
        cam.rotor.normalise();

        Vector3dd position;
        int offset = CAM_MODEL_SIZE * scene->cameras.size();

        position.loadFrom(&in[3 * obs.point + offset]);

        Vector3dd modeled = (cam * position).normalised();
        Vector3dd obsvec  = obs.obs.normalised();

        Vector3dd error = modeled - obsvec;
        error.storeTo(&out[3 * obsId]);
    }

}

vector<double> SillyCost::sceneToModel(const SimplifiedScene& scene)
{
    vector<double> out;
    out.resize(getInputNumber(&scene));

    for (size_t i = 0; i < scene.cameras.size(); i++ )
    {
        Affine3DQ transform = scene.cameras[i].toMockAffine3D();
        transform.shift.storeTo(&out[CAM_MODEL_SIZE * i]);
        transform.rotor.storeTo(&out[CAM_MODEL_SIZE * i + Vector3dd::LENGTH]);
    }

    int offset = scene.cameras.size() * CAM_MODEL_SIZE;

    for (size_t i = 0; i < scene.points.size(); i++ )
    {
        scene.points[i].storeTo(&out[i * Vector3dd::LENGTH + offset]);
    }

    return out;
}

void SillyCost::sceneFromModel(SimplifiedScene &scene, const vector<double> &model)
{
    for (size_t i = 0; i < scene.cameras.size(); i++ )
    {
        Affine3DQ transform;
        transform.shift.loadFrom(&model[i * CAM_MODEL_SIZE]);
        transform.rotor.loadFrom(&model[i * CAM_MODEL_SIZE + Vector3dd::LENGTH]);

        scene.cameras[i] = CameraLocationData::FromMockAffine3D(transform);
    }

    int offset = scene.cameras.size() * CAM_MODEL_SIZE;
    for (size_t i = 0; i < scene.points.size(); i++ )
    {
        scene.points[i].loadFrom(&model[i * Vector3dd::LENGTH +  offset]);
    }



}

#if 0
void SillyCost::sceneToModel(const FixtureScene *scene, vector<double> &model)
{
    CameraFixture *fixture = scene->fixtures()[0];
    for (size_t i = 0; i < fixture->cameras.size(); i++)
    {


    }
}

void SillyCost::sceneFromModel(FixtureScene *scene, const vector<double> &model)
{
    CameraFixture *fixture = scene->fixtures()[0];
    for (size_t i = 0; i < fixture->cameras.size(); i++)
    {


    }
}
#endif

void ExtrinsicsPlacer::place(FixtureScene *scene)
{
    SYNC_PRINT(("ExtrinsicsPlacer::place(FixtureScene *scene)\n"));
    scene->integrityRelink();

    SimplifiedScene S;
    vector<int> idToScene;

    CameraFixture *fixture = scene->fixtures()[0];

    for (size_t i = 0; i < fixture->cameras.size(); i++)
    {
        CameraModel w = fixture->cameras[i]->getWorldCameraModel();
        S.cameras.push_back(w.extrinsics);
    }
    for (size_t i = 0; i < scene->featurePoints().size(); i++)
    {
        SceneFeaturePoint *p = scene->featurePoints()[i];
        /* One more time manual triangulation */

        if (p->observations.size() < 2) {
            SYNC_PRINT(("Point %s: rejected beacause it have few observations\n", p->name.c_str()));
            continue;
        }

        auto it = p->observations.begin();

        if (params.triangulateOnSphere())
        {
            Sphere3d sphere(Vector3dd::Zero(), params.skydomeSize());
            SceneObservation &obs1 = (*it).second;
            Ray3d ray1 = obs1.getFullRay();

            double t1,t2;
            sphere.intersectRayHelper(ray1, t1, t2);

            double t = std::max(std::max(t1, t2), 0.0);

            Vector3dd x = ray1.getPoint(t);
            cout << "Triangulated on sphere" << p->reprojectedPosition << " to " << x << endl;
            p->reprojectedPosition = x;
            p->hasKnownReprojectedPosition = true;

        } else {
            SceneObservation &obs1 = (*it).second;
            it++;
            SceneObservation &obs2 = (*it).second;

            Ray3d ray1 = obs1.getFullRay();
            Ray3d ray2 = obs2.getFullRay();


            Vector3dd coef = ray1.intersectCoef(ray2);

            if (coef[0] < 0 || coef[1] < 0 ) {
                SYNC_PRINT(("Point %s: rejected beacause it is triangulated behind cameras \n", p->name.c_str()));
                continue;
            }

            Vector3dd x1 = ray1.getPoint(coef[0]);
            Vector3dd x2 = ray2.getPoint(coef[1]);
            Vector3dd x = (x1 + x2) / 2.0;

            cout << "Triangulated " << p->reprojectedPosition << " to " << x << endl;
            p->reprojectedPosition = x;
            p->hasKnownReprojectedPosition = true;
        }

        S.points.push_back(p->reprojectedPosition);
        idToScene.push_back(i);

        for (auto &obsIt: p->observations)
        {
            FixtureCamera    *cam = obsIt.first;
            SceneObservation &obs = obsIt.second;
            Vector3dd ray = obs.getRay();
            SimplifiedScene::Observation simpleObs((int)cam->sequenceNumber, (int)(S.points.size() - 1), ray);
            S.obs.push_back(simpleObs);
        }

    }

    cout << "Simplified Scene" << endl;
    cout << S << endl;
    cout << "Camera Model Size" << SillyCost::CAM_MODEL_SIZE << endl;

    SillyCost F(&S);
    SillyNormalizer N(&S);

    vector<double> input = SillyCost::sceneToModel(S);
    vector<double> outputs;
    outputs.resize(F.outputs, 0);

    LevenbergMarquardt lmFit;

    lmFit.maxIterations = 1001;
    lmFit.maxLambda = 10e80;
    lmFit.fTolerance = 1e-19;
    lmFit.xTolerance = 1e-19;

    lmFit.lambdaFactor = 4;
    lmFit.traceCrucial  = true;
    lmFit.traceProgress = true;
    lmFit.trace         = true;

    lmFit.f = &F;
    lmFit.normalisation = &N;


    vector<double> result = lmFit.fit(input, outputs);

    SillyCost::sceneFromModel(S, result);

    for (size_t i = 0; i < S.cameras.size(); i++)
    {
        Affine3DQ transform = S.cameras[i].toAffine3D();
        fixture->cameras[i]->setWorldLocation(transform);
    }

    for (size_t i = 0; i < S.points.size(); i++)
    {

        scene->featurePoints()[idToScene[i]]->position = S.points[i];
    }


    cout << "Input :" << input << endl;
    cout << "Result:" << result << endl;
}


}

