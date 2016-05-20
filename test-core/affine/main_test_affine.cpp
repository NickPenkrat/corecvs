/**
 * \file main_test_affine.cpp
 * \brief This is the main file for the test affine 
 *
 * \date Apr 24, 2011
 * \author alexander
 *
 * \ingroup autotest  
 */
#ifndef TRACE
//#define TRACE
#endif

#include <vector>
#include <stdint.h>
#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "vector3d.h"
#include "affine.h"
#include "mathUtils.h"
#include "preciseTimer.h"
#include "eulerAngles.h"

using namespace corecvs;

TEST(Affine, testRotations)
{
    Vector3dd vx(1.0,0.0,0.0);
    Affine3DM  aym = Affine3DM::RotationY(degToRad(135.0));
    Affine3DQ  ayq = Affine3DQ::RotationY(degToRad(135.0));
    Quaternion byq = Quaternion::RotationY(degToRad(135.0));
    Quaternion cyq = Quaternion(Vector3dd(0.0,1.0,0.0), degToRad(135.0));


    Vector3dd t1m = aym * vx;
    Vector3dd t1q = ayq * vx;
    Vector3dd t2q = byq * vx;
    Vector3dd t3q = cyq * vx;

    Vector3dd r1 = Vector3dd(-1.0 / sqrt(2.0), 0, -1.0 / sqrt(2.0));
    CORE_ASSERT_TRUE_P(t1m.notTooFar(r1, 1e-8), (" Y rotation returned a mistake with Matrix Affine"));
    CORE_ASSERT_TRUE_P(t1q.notTooFar(r1, 1e-8), (" Y rotation returned a mistake with Quaternion Affine"));
    CORE_ASSERT_TRUE_P(t2q.notTooFar(r1, 1e-8), (" Y rotation returned a mistake with Clean Quaternion"));
    CORE_ASSERT_TRUE_P(t3q.notTooFar(r1, 1e-8), (" Y rotation returned a mistake with Clean Quaternion"));
}

TEST(Affine, foo)
{
    auto Q = Quaternion::FromMatrix(corecvs::Matrix33(0, -1, 0, 0, 0, -1, 1, 0, 0));
    std::cout << Q.toMatrix() << std::endl;
}

TEST(Affine, testMatrixToQuaternion)
{
    const int TEST_SIZE = 9;
    vector<Vector3dd> axis(TEST_SIZE);
    double angle[TEST_SIZE];

    angle[0] = 0.01;
    axis [0] = Vector3dd(1.0, 1.0, 1.0);
    angle[1] = 0.01;
    axis [1] = Vector3dd(1.0, 0.0, 0.0);
    angle[2] = 0.01;
    axis [2] = Vector3dd(0.0, 1.0, 0.0);
    angle[3] = 0.01;
    axis [3] = Vector3dd(0.0, 0.0, 1.0);
    angle[4] = 0.7;
    axis [4] = Vector3dd(0.0, 1.0, 1.0);
    angle[5] = -0.01;
    axis [5] = Vector3dd(1.0, 0.0, 0.0);
    angle[6] = -0.01;
    axis [6] = Vector3dd(0.0, 1.0, 0.0);
    angle[7] = -0.01;
    axis [7] = Vector3dd(0.0, 0.0, 1.0);
    angle[8] = -0.7;
    axis [8] = Vector3dd(0.0, 1.0, 1.0);


    for (int i = 0; i < TEST_SIZE; i++)
    {
        Quaternion Q = Quaternion::Rotation(axis[i], angle[i]);
        Matrix33 M = Q.toMatrix();
        Quaternion Q1 = Quaternion::FromMatrix(M);
        std::cout << "Case " << i << std::endl;
        std::cout << Q << " l= " << Q.l2Metric() << std::endl;
        //cout << M << endl;
        std::cout << Q1 << " l= " << Q1.l2Metric() << std::endl;
        ASSERT_TRUE(Q.notTooFar(Q1, 1e-7));
    }

}

TEST(Affine, testEulerAngles)
{
    CameraAnglesLegacy anglesCam(0.7, 0.4, 0.1);
    Matrix33 matrixCam = anglesCam.toMatrix();
    Quaternion quatCam = Quaternion::FromMatrix(matrixCam);
    CameraAnglesLegacy anglesCam1 = CameraAnglesLegacy::FromQuaternion(quatCam);


    std::cout << "A:("  << anglesCam.pitch() << ", "
                        << anglesCam.yaw()   << ", "
                        << anglesCam.roll() << ")" << std::endl;

    std::cout << "M:"   << std::endl << matrixCam << std::endl;
    std::cout << "Q:"   << std::endl << quatCam << std::endl;

    std::cout << "A:("  << anglesCam1.pitch() << ", "
                        << anglesCam1.yaw()   << ", "
                        << anglesCam1.roll() << ")" << std::endl;
}
