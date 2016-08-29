#include "rgb24Buffer.h"
#include "raytraceRenderer.h"
#include "raytraceObjects.h"
#include "raytrace/materialExamples.h"
#include "bmpLoader.h"
#include "preciseTimer.h"
#include "bufferFactory.h"
#include "sdfRenderable.h"


void raytrace_scene_large( void )
{
      SYNC_PRINT(("raytrace_scene_large( void )\n"));


      int h = 1500;
      int w = 1500;
      RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());

      RaytraceRenderer renderer;
      renderer.setProjection(new PinholeCameraIntrinsics(
                  Vector2dd(w, h),
                  degToRad(60.0)));
      renderer.position = Affine3DQ::Identity();
      renderer.sky = new RaytraceableSky1();

      RaytraceablePointLight light1(RGBColor::White() .toDouble(), Vector3dd(  0, -190, 150));
      RaytraceablePointLight light2(RGBColor::Yellow().toDouble(), Vector3dd(-120, -70,  50));

      SDFRenderable object;
      object.F = [](Vector3dd v) {
          double step = 200;
          Vector3dd c1 = Vector3dd(0,-30, 150.0);
          Vector3dd c2 = Vector3dd(0, 40, 150.0);

          double d1 = 20.0 - sqrt((v - c1).sumAllElementsSq());
          double d2 = 20.0 - sqrt((v - c2).sumAllElementsSq());

          if (fabs(d1) > fabs(d2))
          {
              return d2;
          }
          return d1;
      };

      SDFRenderable object1;
      object1.F = [](Vector3dd v) {
          double step = 200;
          Vector3dd c1 = Vector3dd(50,-15, 150.0);
          Vector3dd c2 = Vector3dd(50, 15, 150.0);

          double d1 = 20.0 - sqrt((v - c1).sumAllElementsSq());
          double d2 = 20.0 - sqrt((v - c2).sumAllElementsSq());

          return -1 / ((-1 / d1) + (-1 / d2));
      };



      object.name = "Sphere1";
      object.color = RGBColor::Red().toDouble();
      object.material = MaterialExamples::bumpy();

      object1.name = "Sphere2";
      object1.color = RGBColor::Red().toDouble();
      object1.material = MaterialExamples::bumpy();


      RaytraceableSphere sphere2(Sphere3d(Vector3dd(-80,0, 250.0), 50.0));
      sphere2.name = "Sphere2";
      sphere2.color = RGBColor::Red().toDouble();
      sphere2.material = MaterialExamples::bumpy();

      RaytraceableUnion scene;
      scene.elements.push_back(&object);
      scene.elements.push_back(&object1);
      scene.elements.push_back(&sphere2);

      renderer.object = &scene;
      renderer.lights.push_back(&light1);
      renderer.lights.push_back(&light2);

      renderer.trace(buffer);

      BMPLoader().save("trace-noise.bmp", buffer);

      delete_safe(buffer);
}
