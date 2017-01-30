/**
 * \file main_test_multicamera.cpp
 * \brief This is the main file for the test multicamera 
 *
 * \date янв 27, 2017
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "gtest/gtest.h"

#include "global.h"
#include "multicameraTriangulator.h"
#include "propertyListVisitor.h"
#include "mesh3d.h"

using namespace std;
using namespace corecvs;


TEST(multicamera, testMulticameraSerilize)
{
    cout << "Starting test <multicamera>" << endl;
    MulticameraTriangulator M;
    M.addCamera(Matrix44(1.0), Vector2dd::OrtX());
    M.addCamera(Matrix44(2.0), Vector2dd::OrtY());
    M.addCamera(Matrix44::RotationX(degToRad(10)), Vector2dd(5.0, 6.0));
    M.addCamera(Matrix44::RotationY(degToRad(10)), Vector2dd(1.0));


    PropertyList list;
    PropertyListWriterVisitor writer(&list);
    M.accept<PropertyListWriterVisitor>(writer);
    cout << "Store 1" << endl;
    list.save(cout);

    MulticameraTriangulator M1;
    PropertyListReaderVisitor reader(&list);
    M1.accept<PropertyListReaderVisitor>(reader);

    PropertyList list1;
    PropertyListWriterVisitor writer1(&list1);
    M1.accept<PropertyListWriterVisitor>(writer1);
    cout << "Store 2" << endl;
    list1.save(cout);
}

TEST(multicamera, test702)
{
    MulticameraTriangulator M;
    PropertyListReaderVisitor reader("data/dumpBUILD11_LEFT.txt");
    M.accept<PropertyListReaderVisitor>(reader);

    bool ok = false;

    Vector3dd initial = M.triangulate(&ok);
    if (!ok) {
        SYNC_PRINT(("SceneFeaturePoint::triangulate(): initail guess unable to obtain\n"));
    }
    Vector3dd res = M.triangulateLM(initial, &ok);
    if (!ok) {
        SYNC_PRINT(("SceneFeaturePoint::triangulate(): LM guess unable to obtain\n"));
    }

    Matrix33 covInv1 = M.getCovarianceInvEstimation(initial);
    Matrix33 covInv2 = M.getCovarianceInvEstimation(res);

    cout << "Initial:" << initial << endl;
    cout << "Result :" << res << endl;


     cout << "Covariance at initail:\n" << covInv1 << endl;
     cout << "Covariance at result :\n" << covInv2 << endl;

}

TEST(multicamera, test702draw)
{
    MulticameraTriangulator M;
    PropertyListReaderVisitor reader("data/dumpBUILD11_LEFT.txt");
    M.accept<PropertyListReaderVisitor>(reader);

    Mesh3D mesh;
    for (int i = 0; i < M.P.size(); i++)
    {
        Matrix44  P  = M.P [i];
        Vector2dd xy = M.xy[i];

    }

}

