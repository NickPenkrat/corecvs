#include "rgb24Buffer.h"
#include "raytraceRenderer.h"
#include "raytraceObjects.h"
#include "raytrace/materialExamples.h"
#include "bmpLoader.h"
#include "preciseTimer.h"
#include "bufferFactory.h"

void raytrace_scene_pole( void )
{
    SYNC_PRINT(("raytrace_scene_pole( void )\n"));
    PreciseTimer timer = PreciseTimer::currentTime();

    int h = 500;
    int w = 500;
    RaytraceRenderer renderer;
    renderer.intrisics = PinholeCameraIntrinsics(
                Vector2dd(w, h),
                degToRad(60.0));
    renderer.position = Affine3DQ::Identity();
    //renderer.sky = new RaytraceableSky1();

    RGB24Buffer *cubemap = BufferFactory::getInstance()->loadRGB24Bitmap("cubemap.bmp");
    //cout << "Loaded cubemap" << cubemap->getSize() << endl;
    RaytraceableCubemap cubeMaterial(cubemap);
    renderer.sky = &cubeMaterial;

    RaytraceablePointLight light1(RGBColor::White() .toDouble(), Vector3dd(  0, -190, 150));
    RaytraceablePointLight light2(RGBColor::Yellow().toDouble(), Vector3dd(-120, -70,  50));

    RaytraceableUnion scene;

    for ( int i = 0; i < 7; i ++)
    {
        RaytraceableCylinder *object = new RaytraceableCylinder;
        object->h = 20;
        object->r = 20;
        object->p = Vector3dd(0, -10 + i * object->h, 700);

        object->name = "Cylinder";
        object->color = (i % 2) ? RGBColor::Black().toDouble() : RGBColor::White().toDouble();
        object->material = NULL;
        //object->material = new RaytraceableChessMaterial(5.0);
        //object->material = (i % 2) ? MaterialExamples::ex1() : MaterialExamples::ex2();

        scene.elements.push_back(object);
    }

    RaytraceableSphere sphere2(Sphere3d(Vector3dd(-80,0, 250.0), 20.0));
    sphere2.name = "Sphere2";
    sphere2.color = RGBColor::Red().toDouble();
    sphere2.material = MaterialExamples::bumpy();

    scene.elements.push_back(&sphere2);

    renderer.object = &scene;
    renderer.lights.push_back(&light1);
    renderer.lights.push_back(&light2);

    for (int i = 0; i < 8; i++) {
        RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());
        double a = degToRad(360.0 / 150 * i);
        Vector3dd dir(0, sin(a), cos(a));
        //renderer.position = Affine3DQ::Shift(dir * 200.0) * Affine3DQ::RotationX(a) *  Affine3DQ::Shift(0, 0, -200.0);
        renderer.position = Affine3DQ::Shift(Vector3dd(0, i * 3 , 0 )) * Affine3DQ::RotationY(0.0);

        renderer.supersample = true;
        renderer.sampleNum = 1000;
        renderer.trace(buffer);

        char name[100];
        snprintf2buf(name, "trace-cylinder%d.bmp", i);
        BMPLoader().save(name, buffer);
        delete_safe(buffer);
    }

    SYNC_PRINT(("Processed: %lf seconds elapsed\n", timer.usecsToNow() / 1e6 ));
}
