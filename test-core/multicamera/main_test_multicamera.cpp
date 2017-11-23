/**
 * \file main_test_multicamera.cpp
 * \brief This is the main file for the test multicamera 
 *
 * \date янв 27, 2017
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <vector>
#include <iostream>
#include "gtest/gtest.h"

#include "core/utils/global.h"
#include "core/rectification/multicameraTriangulator.h"
#include "core/utils/visitors/propertyListVisitor.h"

#include "core/geometry/mesh3d.h"

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

     /* Attempt to remove cameras one by one */
     cout << "Attempt to remove cameras one by one " << endl;
     for (int skip = 0; skip < M.getSize(); skip++)
     {
         cout << "Skipping " << skip << " measurement" << endl;
         vector<bool> mask(M.getSize(), true);
         mask[skip] = false;
         for (size_t i = 0; i < mask.size(); i++)
         {
             cout << (mask[i] ? "X" : "O");
         }
         cout << endl;

         MulticameraTriangulator M1 = M.subset(mask);
         cout << "We have " << M1.getSize() << " measuremets" << endl;


         Vector3dd initial = M1.triangulate(&ok);
         if (!ok) {
             SYNC_PRINT(("SceneFeaturePoint::triangulate(): initail guess unable to obtain\n"));
         }
         Vector3dd res = M1.triangulateLM(initial, &ok);
         if (!ok) {
             SYNC_PRINT(("SceneFeaturePoint::triangulate(): LM guess unable to obtain\n"));
         }
         cout << "   Initial:" << initial << endl;
         cout << "   Result :" << res << endl;
     }


     /* Attempt to take pairs of cameras */
     cout << "Attempt to take pairs of cameras" << endl;

     for (int skip1 = 0; skip1 < M.getSize(); skip1++)
     {

         for (int skip2 = skip1 + 1; skip2 < M.getSize(); skip2++)
         {
             cout << "Selecting " << skip1 << " and " << skip2 << " measurement" << endl;
             vector<bool> mask(M.getSize(), false);
             mask[skip1] = true;
             mask[skip2] = true;

             for (size_t i = 0; i < mask.size(); i++)
             {
                 cout << (mask[i] ? "X" : "O");
             }
             cout << endl;

             MulticameraTriangulator M1 = M.subset(mask);
             cout << "We have " << M1.getSize() << " measuremets" << endl;


             Vector3dd initial = M1.triangulate(&ok);
             if (!ok) {
                 SYNC_PRINT(("SceneFeaturePoint::triangulate(): initail guess unable to obtain\n"));
             }
             Vector3dd res = M1.triangulateLM(initial, &ok);
             if (!ok) {
                 SYNC_PRINT(("SceneFeaturePoint::triangulate(): LM guess unable to obtain\n"));
             }
             cout << "   Initial:" << initial << endl;
             cout << "   Result :" << res << endl;

         }
    }

}

#if 0
TEST(multicamera, test702draw)
{
    MulticameraTriangulator M;
    PropertyListReaderVisitor reader("data/dumpBUILD11_LEFT.txt");
    M.accept<PropertyListReaderVisitor>(reader);

    Mesh3D mesh;
    for (size_t i = 0; i < M.P.size(); i++)
    {
        Matrix44  P  = M.P [i];
        Vector2dd xy = M.xy[i];

    }

}
#endif
