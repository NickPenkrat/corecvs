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
        rotor.x() = out[7 * camId + 3];
        rotor.y() = out[7 * camId + 4];
        rotor.z() = out[7 * camId + 5];
        rotor.t() = out[7 * camId + 6];

        rotor.normalise();

        out[7 * camId + 3] = rotor.x();
        out[7 * camId + 4] = rotor.y();
        out[7 * camId + 5] = rotor.z();
        out[7 * camId + 6] = rotor.t();
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
        cam.shift.x() = in[7 * obs.cam + 0];
        cam.shift.y() = in[7 * obs.cam + 1];
        cam.shift.z() = in[7 * obs.cam + 2];

        cam.rotor.x() = in[7 * obs.cam + 3];
        cam.rotor.y() = in[7 * obs.cam + 4];
        cam.rotor.z() = in[7 * obs.cam + 5];
        cam.rotor.t() = in[7 * obs.cam + 6];
        cam.rotor.normalise();

        Vector3dd position;
        int offset = 7 * scene->cameras.size();

        position.x() = in[3 * obs.point + 0 + offset];
        position.y() = in[3 * obs.point + 1 + offset];
        position.z() = in[3 * obs.point + 2 + offset];


        Vector3dd modeled = (cam * position).normalised();
        Vector3dd obsvec  = obs.obs.normalised();

        out[3 * obsId + 0] = modeled.x() - obsvec.x();
        out[3 * obsId + 1] = modeled.y() - obsvec.y();
        out[3 * obsId + 2] = modeled.z() - obsvec.z();

    }

}

vector<double> SillyCost::sceneToModel(const SimplifiedScene& scene)
{
    vector<double> out;
    out.resize(scene.getInputNumber());

    for (size_t i = 0; i < scene.cameras.size(); i++ )
    {
        Affine3DQ transform = scene.cameras[i].toMockAffine3D();
        out[i*7 + 0] = transform.shift.x();
        out[i*7 + 1] = transform.shift.y();
        out[i*7 + 2] = transform.shift.z();

        out[i*7 + 3] = transform.rotor.x();
        out[i*7 + 4] = transform.rotor.y();
        out[i*7 + 5] = transform.rotor.z();
        out[i*7 + 6] = transform.rotor.t();
    }

    int offset = scene.cameras.size() * 7;
    for (size_t i = 0; i < scene.points.size(); i++ )
    {
        out[i*3 + 0 + offset] = scene.points[i].x();
        out[i*3 + 1 + offset] = scene.points[i].y();
        out[i*3 + 2 + offset] = scene.points[i].z();
    }

    return out;
}

void SillyCost::sceneFromModel(SimplifiedScene &scene, const vector<double> &model)
{
    for (size_t i = 0; i < scene.cameras.size(); i++ )
    {
        Affine3DQ transform;
        transform.shift.x() = model[i*7 + 0];
        transform.shift.y() = model[i*7 + 1];
        transform.shift.z() = model[i*7 + 2];

        transform.rotor.x() = model[i*7 + 3];
        transform.rotor.y() = model[i*7 + 4];
        transform.rotor.z() = model[i*7 + 5];
        transform.rotor.t() = model[i*7 + 6];

        scene.cameras[i] = CameraLocationData::FromMockAffine3D(transform);
    }

    int offset = scene.cameras.size() * 7;
    for (size_t i = 0; i < scene.points.size(); i++ )
    {
        scene.points[i].x() = model[i*3 + 0 + offset];
        scene.points[i].y() = model[i*3 + 1 + offset];
        scene.points[i].z() = model[i*3 + 2 + offset];
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

        cout << "Triangulated " << p->position << " to " << x << endl;
        p->position = x;

        S.points.push_back(p->position);
        idToScene.push_back(i);

        for (auto &obsIt: p->observations)
        {
            FixtureCamera    *cam = obsIt.first;
            SceneObservation &obs = obsIt.second;
            Vector3dd ray = obs.getRay();
            SimplifiedScene::Observation simpleObs((int)cam->sequenceNumber, (int)i, ray);
            S.obs.push_back(simpleObs);
        }

    }

    cout << "Simplified Scene" << endl;
    cout << S << endl;

    SillyCost F(&S);
    SillyNormalizer N(&S);

    vector<double> input = SillyCost::sceneToModel(S);
    vector<double> outputs;
    outputs.resize(F.outputs, 0);

    LevenbergMarquardt lmFit;

    lmFit.maxIterations = 10000001;
    lmFit.maxLambda = 10e80;
    lmFit.fTolerance = 1e-19;
    lmFit.xTolerance = 1e-19;

    lmFit.lambdaFactor = 8;
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

