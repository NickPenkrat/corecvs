#include <abstractFileCaptureSpinThread.h>
#include <cmath>
#include <stdio.h>

#include <iostream>
#include <random>

#include "fixtureScene.h"
#include "calibrationHelpers.h"
#include "mesh3d.h"

using namespace std;
using namespace corecvs;

FixtureScene *genTest()
{
    std::mt19937 rng;
    std::uniform_real_distribution<double> camPos(-100,100);
    std::normal_distribution<double> pointPos(0, 100);

    double radius = 0.1;

    const int FIXTURE_NUM = 128;
    const int CAM_NUM = 16;
    const int POINT_NUM = 60000 * 4;

    FixtureScene *scene = new FixtureScene;

    CameraModel model;
    model.intrinsics.principal.x() = 320;
    model.intrinsics.principal.y() = 240;
    model.intrinsics.focal.x() = 589;
    model.intrinsics.focal.y() = 589;
    model.intrinsics.size = Vector2dd(640, 480);


    for (int idf = 0; idf < FIXTURE_NUM; idf++)
    {
        CameraFixture *fixture = scene->createCameraFixture();
        fixture->setLocation(Affine3DQ::Shift(camPos(rng), camPos(rng), camPos(rng) / 10.0));

        for (int ic = 0; ic < CAM_NUM; ic++)
        {
            FixtureCamera *camera = scene->createCamera();
            char buffer[100];
            snprintf2buf(buffer, "camera %d", ic);
            camera->nameId = buffer;

            camera->copyModelFrom(model);

            Affine3DQ position =
                    Affine3DQ::RotationX(degToRad(180)) *
                    Affine3DQ::RotationZ(degToRad(360.0 / CAM_NUM) * ic) * Affine3DQ::Shift(radius, 0, 0);

            scene->positionCameraInFixture(fixture, camera, position);
            scene->addCameraToFixture(camera, fixture);

        }

        SYNC_PRINT(("Creating station: %d\n", idf));

    }

    for (int ip = 0; ip < POINT_NUM; ip++)
    {
        SceneFeaturePoint *point = scene->createFeaturePoint();
        point->setPosition(Vector3dd(pointPos(rng), pointPos(rng), pointPos(rng)));
    }

    SYNC_PRINT(("Projecting points..."));

    scene->projectForward(SceneFeaturePoint::POINT_ALL);
    SYNC_PRINT(("...Done\n"));

    return scene;
}

int main (void)
{
#if 0
    cout << "Starting test <jit>" << endl;
    cout << "This test is x64 and GCC only" << endl;

#if defined (__GNUC__) && __x86_64

    double sin_a, cos_a, a = 0.5;
    asm ("fldl %2;"
         "fsincos;"
         "fstpl %1;"
         "fstpl %0;" : "=m"(sin_a), "=m"(cos_a) : "m"(a));
    printf("sin(29°) = %lf, cos(29°) = %lf\n", sin_a, cos_a);

#endif

    double sin_b;
    double cos_b;
    double b = 0.5;

    sin_b = sin(b);
    cos_b = cos(b);

    double sum = sin_b + cos_b;
    double diff = cos_b - sin_b;


    printf("V: sin(29°) = %lf, cos(29°) = %lf\n", sin_b, cos_b);
    printf("V: sum = %lf, diff = %lf\n", sum, diff);

    cout << "Test <jit> PASSED" << endl;
#endif

    FixtureScene *scene = genTest();

    scene->dumpInfo();

    Mesh3D dump;
    dump.switchColor();
    CalibrationHelpers().drawScene(dump, *scene, 0.2);
    dump.dumpPLY("large.ply");


    return 0;
};
