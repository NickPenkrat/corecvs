/**
 * \file main_test_calstructs.cpp
 * \brief This is the main file for the test calstructs 
 *
 * \date Oct 02, 2015
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "mesh3d.h"
#include "calibrationScene.h"
#include "calibrationHelpers.h"

using namespace std;
using namespace corecvs;


/**
 *
 *
 *
 *
 *
 *
 *
 *
 **/
TEST(calstructs, testCameraModel)
{
    Mesh3D mesh;
    mesh.switchColor();
    mesh.setColor(RGBColor::gray(0x7F));
    mesh.addIcoSphere(Vector3dd(0.0), 0.1,  2);
    mesh.faces.clear();
    mesh.facesColor.clear();


    mesh.setColor(RGBColor::Red());
    mesh.addLine(Vector3dd(0.0), Vector3dd::OrtX());
    mesh.setColor(RGBColor::Green());
    mesh.addLine(Vector3dd(0.0), Vector3dd::OrtY());
    mesh.setColor(RGBColor::Blue());
    mesh.addLine(Vector3dd(0.0), Vector3dd::OrtZ());


    CameraModel model(PinholeCameraIntrinsics(Vector2dd(100.0,100.0), degToRad(45.0)),
                      LocationData(
                          Vector3dd(3.0,0.0,0.0),
                          Quaternion::RotationY(degToRad(-45.0))
                      )
                 );

    Vector3dd point(5.0, 2.0, 5.0);

    /* Direction in camera frame */
    Vector3dd dirInCam = model.dirToPoint(point);


    cout << "Direction        :" << dirInCam << endl;

    Vector2dd projection1 =  model.intrinsics.project(dirInCam);



    Vector2dd projection  = model.project(point);
    //Vector2dd projection1 = model.project(rayInCam.getPoint(1.0));
    cout << "Direct Projection:" << projection << endl;
    cout << "Ray    Projection:" << projection1 << endl;

   // ASSERT_TRUE(rayInCam.p.notTooFar(model.extrinsics.position, 1e-7));

   // ASSERT_TRUE(rayInCam.projectOnRay(point).notTooFar(point, 1e-7));
    ASSERT_TRUE(projection.notTooFar(projection1));

    CalibrationHelpers().drawCamera(mesh, model, 2.0);

    mesh.dumpPLY("cammodel.ply");
    cout << "Test <calstructs> PASSED" << endl;
}
