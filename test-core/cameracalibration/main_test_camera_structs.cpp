#ifndef ASSERTS
#define ASSERTS
#endif

#include "gtest/gtest.h"

#include "global.h"
#include "vector4d.h"

#include "calibrationCamera.h"
#include "calibrationLocation.h"

#include <random>

const int DEFAULT_SEED = 777;
const int RNG_RETRIES = 8192;

TEST(CalibrationStructsTest, testFundamentalProvider)
{
    std::mt19937 rng((std::random_device())());
    std::uniform_real_distribution<double> unif(-1e3, 1e3);

    int validCnt = 0;
    PinholeCameraIntrinsics intrinsics(1.0, 2.0, 3.0, 4.0, 5.0);
    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        CameraModel camera1(
            intrinsics,
            CameraLocationData(
                Vector3dd(unif(rng), unif(rng), unif(rng)),
                Quaternion(unif(rng), unif(rng), unif(rng), unif(rng)).normalised()));
        CameraModel camera2(
            intrinsics,
            CameraLocationData(
                Vector3dd(unif(rng), unif(rng), unif(rng)),
                Quaternion(unif(rng), unif(rng), unif(rng), unif(rng)).normalised()));
        auto E  = camera1.fundamentalTo(camera2);
        auto F  = CameraModel::Fundamental(camera1.getCameraMatrix(), camera2.getCameraMatrix());

        auto vv = E.rank2Nullvectors();
        ASSERT_NEAR(!(E * vv[1]), 0.0, 1e-6 * !vv[1]);
        ASSERT_NEAR(!(vv[0] * E), 0.0, 1e-6 * !vv[0]);

        for (int j = 0; j < RNG_RETRIES; ++j)
        {
            corecvs::Vector3dd pt(unif(rng), unif(rng), unif(rng));
            if (camera1.isVisible(pt) && camera2.isVisible(pt))
            {
                auto p1 = camera1.project(pt);
                auto p2 = camera2.project(pt);
                corecvs::Vector3dd L(p1[0], p1[1], 1.0);
                corecvs::Vector3dd R(p2[0], p2[1], 1.0);

                auto el = E * R;
                auto fl = F * R;
                ASSERT_NEAR(L & (E * R) / std::sqrt(el[0] * el[0] + el[1] * el[1]), 0.0, 1e-6);
                ASSERT_NEAR(L & (F * R) / std::sqrt(fl[0] * fl[0] + fl[1] * fl[1]), 0.0, 1e-6);
                validCnt++;
            }
        }
    }
    ASSERT_TRUE(validCnt > RNG_RETRIES);

}

TEST(CalibrationStructsTest, testEssentialProvider)
{
    std::mt19937 rng(DEFAULT_SEED);
    std::uniform_real_distribution<double> unif(-1e3, 1e3);

    int validCnt = 0;
    PinholeCameraIntrinsics intrinsics(1.0, 2.0, 3.0, 4.0, 5.0);
    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        CameraModel camera1(
            intrinsics,
            CameraLocationData(
                Vector3dd(unif(rng), unif(rng), unif(rng)),
                Quaternion(unif(rng), unif(rng), unif(rng), unif(rng)).normalised()));
        CameraModel camera2(
            intrinsics,
            CameraLocationData(
                Vector3dd(unif(rng), unif(rng), unif(rng)),
                Quaternion(unif(rng), unif(rng), unif(rng), unif(rng)).normalised()));
        auto K1 = camera1.intrinsics.getKMatrix33().inv();
        auto K2 = camera2.intrinsics.getKMatrix33().inv();
        auto E  = camera1.essentialTo(camera2);
        for (int j = 0; j < RNG_RETRIES; ++j)
        {
            corecvs::Vector3dd pt(unif(rng), unif(rng), unif(rng));
            if (camera1.isVisible(pt) && camera2.isVisible(pt))
            {
                auto p1 = camera1.project(pt);
                auto p2 = camera2.project(pt);
                corecvs::Vector3dd L(p1[0], p1[1], 1.0);
                corecvs::Vector3dd R(p2[0], p2[1], 1.0);
                L = K1 * L;
                R = K2 * R;
                L /= L[2];
                R /= R[2];
                ASSERT_NEAR(L & (E * R), 0.0, 1e-3);
                validCnt++;
            }
        }
    }
    ASSERT_TRUE(validCnt > RNG_RETRIES);

}

TEST(CalibrationStructsTest, testIntrinsicsStruct)
{
    // Here we test interoperability of intrinsics struct
    // and returned projection matrix
    corecvs::PinholeCameraIntrinsics intrinsics(1.0, 2.0, 3.0, 4.0, 5.0);
    auto M = intrinsics.getKMatrix();

    std::mt19937 rng(DEFAULT_SEED);
    std::uniform_real_distribution<double> unif(-1e3, 1e3);

    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        corecvs::FixedVector<double, 4> src1;
        corecvs::Vector3dd src2;

        for (int j = 0; j < 3; ++j)
            src1[j] =  src2[j] = unif(rng);
        if (src1[2] == 0.0)
            src1[2] = src2[2] = 1.0;

        src1[3] = 1.0;

        auto dst1t = M * src1;
        auto dst2 = intrinsics.project(src2);

        corecvs::Vector2dd dst1(dst1t[0] / dst1t[2], dst1t[1] / dst1t[2]);
        ASSERT_NEAR(dst1[0], dst2[0], 1e-6);
        ASSERT_NEAR(dst1[1], dst2[1], 1e-6);
    }
}

TEST(CalibrationStructsTest, testCameraStruct)
{
    // Here we test interoperability of camera struct
    // and returned projection matrix
    corecvs::PinholeCameraIntrinsics intrinsics(1.0, 2.0, 3.0, 4.0, 5.0);
    corecvs::CameraModel camera(
            intrinsics,
            CameraLocationData(
                corecvs::Vector3dd(6.0, 7.0, 8.0),
                corecvs::Quaternion(0.5, 0.5, 0.5, 0.5)));
    auto M = camera.getCameraMatrix();

    std::mt19937 rng(DEFAULT_SEED);
    std::uniform_real_distribution<double> unif(-1e3, 1e3);

    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        corecvs::FixedVector<double, 4> src1;
        corecvs::Vector3dd src2;

        for (int j = 0; j < 3; ++j)
            src1[j] =  src2[j] = unif(rng);

        src1[3] = 1.0;

        auto dst1t = M * src1;
        auto dst2 = camera.project(src2);

        corecvs::Vector2dd dst1(dst1t[0] / dst1t[2], dst1t[1] / dst1t[2]);
        ASSERT_NEAR(dst1[0], dst2[0], 1e-6);
        ASSERT_NEAR(dst1[1], dst2[1], 1e-6);
    }
}

TEST(CalibrationStructsTest, testIntrinsicsStructisVisible)
{
    corecvs::PinholeCameraIntrinsics intrinsics(1.0, 1.0, 0.5, 0.5, 0.0, corecvs::Vector2dd(1.0, 1.0));
    ASSERT_TRUE (intrinsics.isVisible(corecvs::Vector3dd( 0.0,  0.0, 1.0)));
    ASSERT_FALSE(intrinsics.isVisible(corecvs::Vector3dd( 2.0,  0.0, 1.0)));
    ASSERT_FALSE(intrinsics.isVisible(corecvs::Vector3dd(-2.0,  0.0, 1.0)));
    ASSERT_FALSE(intrinsics.isVisible(corecvs::Vector3dd( 0.0, -2.0, 1.0)));
    ASSERT_FALSE(intrinsics.isVisible(corecvs::Vector3dd( 0.0,  2.0, 1.0)));
}


TEST(CalibrationStructsTest, testStructConversion)
{
    CameraLocationAngles angles = CameraLocationAngles::FromAngles(45, 10, 2);
    Quaternion q  = Quaternion::FromMatrix(angles.toMatrix());
    Quaternion q1 = angles.toQuaternion();



    CameraLocationAngles anglesR = CameraLocationAngles::FromQuaternion(q);
    Quaternion qR = Quaternion::FromMatrix(anglesR.toMatrix());

    cout << "Original:" << std::endl;
    cout << angles << std::endl;
    cout << "Quaternion form1:" << std::endl;
    q.printAxisAndAngle();

    cout << "Quaternion form2:" << std::endl;
    q1.printAxisAndAngle();
    cout << "Restored:" << std::endl;
    cout << anglesR << std::endl;
    qR.printAxisAndAngle();

    ASSERT_TRUE(q.notTooFar(q1, 1e-6));
    ASSERT_TRUE(q.notTooFar(qR, 1e-6));



}
