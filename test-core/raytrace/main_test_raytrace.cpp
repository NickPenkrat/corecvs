/**
 * \file main_test_raytrace.cpp
 * \brief This is the main file for the test raytrace 
 *
 * \date мая 06, 2016
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <fstream>
#include <iostream>
#include "objLoader.h"
#include "bufferFactory.h"
#include "gtest/gtest.h"

#include "global.h"

#include "raytrace/raytraceRenderer.h"
#include "raytrace/raytraceObjects.h"
#include "raytrace/sdfRenderable.h"
#include "raytrace/materialExamples.h"

#include "bmpLoader.h"
#include "meshLoader.h"
#include "preciseTimer.h"
#include "perlinNoise.h"

using namespace std;
using namespace corecvs;


TEST(Raytrace, DISABLED_testRaytraceModifiers)
{
    int h = 1500;
    int w = 1500;
    RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());

    RaytraceRenderer renderer;
    renderer.intrisics = PinholeCameraIntrinsics(
                Vector2dd(w, h),
                degToRad(60.0));
    renderer.position = Affine3DQ::Identity();

    /* Materials */
    RaytraceablePointLight light1(RGBColor::White() .toDouble(), Vector3dd( 0, -190, 150));
    RaytraceablePointLight light2(RGBColor::Yellow().toDouble(), Vector3dd(-120, -70,  50));

    RaytraceableSphere sphere1(Sphere3d(Vector3dd(0,0, 150.0), 50.0));
    sphere1.name = "Sphere1";
    sphere1.color = RGBColor::Red().toDouble();
    sphere1.material = MaterialExamples::glass(0.85);

    RaytraceableSphere sphere2(Sphere3d(Vector3dd(-80,0, 250.0), 50.0));
    sphere2.name = "Sphere1";
    sphere2.color = RGBColor::Red().toDouble();
    sphere2.material = MaterialExamples::bumpy();

    RaytraceableUnion scene;
    scene.elements.push_back(&sphere1);
    scene.elements.push_back(&sphere2);

    renderer.object = &scene;
    renderer.lights.push_back(&light1);
    renderer.lights.push_back(&light2);

    renderer.trace(buffer);

    BMPLoader().save("trace-noise.bmp", buffer);
    delete_safe(buffer);
}

TEST(Raytrace, DISABLED_testRaytraceTextures)
{
    int h = 1500;
    int w = 1500;
    RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());

    RaytraceRenderer renderer;
    renderer.intrisics = PinholeCameraIntrinsics(
                Vector2dd(w, h),
                degToRad(60.0));
    //renderer.position = Affine3DQ::Shift(0, 0, -200);

    /* Ligths */
    RaytraceablePointLight light1(RGBColor::White() .toDouble(), Vector3dd( 0, -190, 150));
    RaytraceablePointLight light2(RGBColor::Yellow().toDouble(), Vector3dd(-120, -70,  50));

    Mesh3DDecorated mesh;
    OBJLoader loader;
    std::ifstream file("body-v2.obj", std::ifstream::in);
    loader.loadOBJ(file, mesh);
    //mesh.transform(Matrix44::Shift(0, 50, 2180) * Matrix44::Scale(1) * Matrix44::RotationZ(degToRad(-90)) * Matrix44::RotationY(degToRad(-90)) * Matrix44::RotationX(degToRad(180)));
    mesh.transform(Matrix44::Shift(0, 1200, 1580) * Matrix44::Scale(1.1) * Matrix44::RotationZ(degToRad(-90)) * Matrix44::RotationY(degToRad(-90)) * Matrix44::RotationX(degToRad(180)));

    mesh.dumpInfo();

    RGB24Buffer *texture = BMPLoader().loadRGB("body-v2.bmp");
    if (texture) {
        cout << "Texture Size:" <<  texture->getSize() << endl;
    } else {
        cout << "Can't load texture";
    }

    TextureMaterial textMat;
    textMat.texture = texture;

    RaytraceableOptiMesh roMesh(&mesh);
    roMesh.name = "Rabbit";
    roMesh.color = RGBColor::Blue().toDouble();
    roMesh.material = &textMat;
    roMesh.optimize();

    RaytraceableSphere sphere2(Sphere3d(Vector3dd(-80,0, 250.0), 50.0));
    sphere2.name = "Sphere1";
    sphere2.color = RGBColor::Red().toDouble();
    sphere2.material = MaterialExamples::ex1();

    RaytraceableUnion scene;
    scene.elements.push_back(&roMesh);
    scene.elements.push_back(&sphere2);

    renderer.object = &scene;
    renderer.lights.push_back(&light1);
    renderer.lights.push_back(&light2);

    renderer.trace(buffer);

    BMPLoader().save("trace-noise.bmp", buffer);
    delete_safe(buffer);
}

TEST(Raytrace, testSDF)
{
    SDFRenderable object;
    object.F = [](Vector3dd v) { return 50.0 - (v - Vector3dd(0,0, 150.0)).l2Metric(); };

    RayIntersection ray;
    ray.ray = Ray3d(Vector3dd::OrtZ(), Vector3dd::Zero());
    object.intersect(ray);

    cout << "R:" << ray.ray << endl;
    cout << "P:" << ray.getPoint() << endl;
    cout << "N:" << ray.normal << endl;

    ASSERT_TRUE(ray.t == 100);


    ray.ray = Ray3d(Vector3dd(0.1, 0, 1.0).normalised(), Vector3dd::Zero());
    object.intersect(ray);
    cout << "R:" << ray.ray << endl;
    cout << "P:" << ray.getPoint() << endl;
    cout << "N:" << ray.normal << endl;
}

TEST(Raytrace, testCylinder)
{
    Mesh3D mesh;
    RaytraceableCylinder object;
    /*object.e1 = Vector3dd::OrtX();
    object.e2 = Vector3dd::OrtZ();
    object.n  = Vector3dd::OrtY();*/
    object.h = 50;
    object.r = 20;
    object.p = Vector3dd(0,-10,200);

    Ray3d rays[] = {
      /*  Ray3d(Vector3dd::OrtZ()   , Vector3dd::Zero()),
        Ray3d(Vector3dd::OrtZ()   , Vector3dd(sqrt(object.r), 0, 0)),
        Ray3d(Vector3dd::OrtZ()   , Vector3dd(-19.99, 0, 0)),
        Ray3d(Vector3dd(0, 0.1, 1), Vector3dd(-19.99, 0, 0)),

        Ray3d(Vector3dd(0, 0.1, 1), Vector3dd(-19.99, 0, 0)),*/

        Ray3d(Vector3dd(0, -1, 1), Vector3dd(0, 240, 0)),
    };

    for (int i = 0; i < CORE_COUNT_OF(rays); i++)
    {
        rays[i] = rays[i].normalised();
        RayIntersection ray;        
        ray.ray = rays[i];
        mesh.addLine(rays[i].getPoint(0), rays[i].getPoint(300));
        bool ok = object.intersect(ray);

        cout << i << endl;
        cout << " R:" << ray.ray << endl;
        if (ok) {
        cout << " P:" << ray.getPoint() << endl;
        //cout << " N:" << ray.normal << endl;
        } else  {
            cout << "No Intersecution" << endl;
        }

        mesh.addPoint(ray.getPoint());
    }

    object.toMesh(mesh);
    mesh.dumpPLY("cylinder-int.ply");
}

TEST(Raytrace, testCylinder1)
{
    Mesh3D mesh;
    RaytraceableCylinder object;
    /*object.e1 = Vector3dd::OrtX();
    object.e2 = Vector3dd::OrtZ();
    object.n  = Vector3dd::OrtY();*/
    object.h = 50;
    object.r = 20;
    object.p = Vector3dd(0,-10,200);

    int limit = 100;

    vector<Ray3d> rays;


    for (int i = - limit;  i < limit ; i++)
    {
        for (int j = - limit;  j < limit ; j++)
        {
            /*rays.push_back(Ray3d(Vector3dd(i / (3.0 * limit), j /  (3.0 * limit), 1.0 ), Vector3dd::Zero()));
            rays.back().normalise();*/

            rays.push_back(Ray3d(Vector3dd(i / (2.0 * limit), j /  (2.0 * limit) - 0.4, 1.0 ), Vector3dd(60, 120, 0)));
            rays.back().normalise();
        }
    }

    for (size_t i = 0; i < rays.size(); i++)
    {
        RayIntersection ray;
        ray.ray = rays[i];
        mesh.addLine(rays[i].getPoint(0), rays[i].getPoint(30));
        bool ok = object.intersect(ray);

        cout << i << endl;
        cout << " R:" << ray.ray << endl;
        if (ok) {
        cout << " P:" << ray.getPoint() << endl;
        //cout << " N:" << ray.normal << endl;
        } else  {
            cout << "No Intersecution" << endl;
        }

        mesh.addPoint(ray.getPoint());
    }

    //object.toMesh(mesh);
    mesh.dumpPLY("cylinder-int.ply");
}


TEST(Raytrace, DISABLED_testRaytraceSDF)
{
    int h = 1500;
    int w = 1500;
    RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());

    RaytraceRenderer renderer;
    renderer.intrisics = PinholeCameraIntrinsics(
                Vector2dd(w, h),
                degToRad(60.0));
    renderer.position = Affine3DQ::Identity();

    RaytraceablePointLight light1(RGBColor::White() .toDouble(), Vector3dd(  0, -190, 150));
    RaytraceablePointLight light2(RGBColor::Yellow().toDouble(), Vector3dd(-120, -70,  50));

    SDFRenderable object;
    object.F = [](Vector3dd v) {
        Vector3dd c1 = Vector3dd(0,-30, 150.0);
        Vector3dd c2 = Vector3dd(0, 30, 150.0);

        return  sqrt(400.0 / ((v - c1).sumAllElementsSq() +
                              (v - c2).sumAllElementsSq()));
    };
    object.name = "Sphere1";
    object.color = RGBColor::Red().toDouble();
    object.material = MaterialExamples::bumpy();

    RaytraceableSphere sphere2(Sphere3d(Vector3dd(-80,0, 250.0), 50.0));
    sphere2.name = "Sphere2";
    sphere2.color = RGBColor::Red().toDouble();
    sphere2.material = MaterialExamples::bumpy();

    RaytraceableUnion scene;
    scene.elements.push_back(&object);
    scene.elements.push_back(&sphere2);

    renderer.object = &scene;
    renderer.lights.push_back(&light1);
    renderer.lights.push_back(&light2);

    renderer.trace(buffer);

    BMPLoader().save("trace-noise.bmp", buffer);

    delete_safe(buffer);
}

TEST(Raytrace, DISABLED_testRaytraceCylinder)
{
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

    Cylinder3d cylinder;
    cylinder.c      = Vector3dd(0,0,200);
    cylinder.normal = Vector3dd::OrtY();
    cylinder.height = 30;
    cylinder.r      = 40;

    RaytraceableCylinder object;
    object.h = 50;
    object.r = 20;
    object.p = Vector3dd(0,30,200);

    object.name = "Cylinder";
    object.color = RGBColor::Red().toDouble();
    object.material = new RaytraceableChessMaterial(5.0);
    object.material = MaterialExamples::ex1();

    RaytraceableSphere sphere2(Sphere3d(Vector3dd(-80,0, 250.0), 50.0));
    sphere2.name = "Sphere2";
    sphere2.color = RGBColor::Red().toDouble();
    sphere2.material = MaterialExamples::bumpy();

    RaytraceableUnion scene;
    scene.elements.push_back(&object);
    //scene.elements.push_back(&sphere2);

    renderer.object = &scene;
    renderer.lights.push_back(&light1);
    renderer.lights.push_back(&light2);

    for (int i = 0; i < 150; i++) {
        RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());
        double a = degToRad(360.0 / 150 * i);
        Vector3dd dir(0, sin(a), cos(a));
        //renderer.position = Affine3DQ::Shift(dir * 200.0) * Affine3DQ::RotationX(a) *  Affine3DQ::Shift(0, 0, -200.0);
        renderer.position = Affine3DQ::Shift(Vector3dd(0, i * 3 , 0 )) * Affine3DQ::RotationY(0.0);

        renderer.trace(buffer);

        char name[100];
        snprintf2buf(name, "trace-cylinder%d.bmp", i);
        BMPLoader().save(name, buffer);
        delete_safe(buffer);
    }
}
