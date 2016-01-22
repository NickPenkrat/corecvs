/**
 * \file main_test_camerafixture.cpp
 * \brief This is the main file for the test camerafixture 
 *
 * \date дек. 30, 2015
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "gtest/gtest.h"

#include "global.h"
#include "fixtureScene.h"
#include "printerVisitor.h"


using namespace std;
using namespace corecvs;

TEST(Fixture, testAllocations)
{    
    FixtureScene *scene = new FixtureScene();
    CameraFixture *fixture1 = scene->createCameraFixture();
    fixture1->name = "Fixture1";

    FixtureCamera *camera1 = scene->createCamera();
    camera1->nameId = "Camera1";
    FixtureCamera *camera2 = scene->createCamera();
    camera2->nameId = "Camera2";

    scene->addCameraToFixture(camera1, fixture1);

    scene->dumpInfo(cout);


    delete_safe(scene);
}

TEST(Fixture, testVisitors)
{
    cout << "----------------Ruuning the test-------------" << std::endl;
    FixtureScene *scene = new FixtureScene();
    CameraFixture *fixture1 = scene->createCameraFixture();
    fixture1->name = "Fixture1";

    FixtureCamera *camera1 = scene->createCamera();
    camera1->nameId = "Camera1";
    FixtureCamera *camera2 = scene->createCamera();
    camera2->nameId = "Camera2";

    scene->addCameraToFixture(camera1, fixture1);

    scene->dumpInfo(cout);
    PrinterVisitor visitor;
    scene->accept(visitor);




    delete_safe(scene);
}
