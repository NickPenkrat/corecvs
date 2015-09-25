#ifndef ASSERTS
#define ASSERTS
#endif

#include "gtest/gtest.h"

#include "global.h"

#include "calibrationStructs.h"

#include <random>

const int DEFAULT_SEED = 777;
const int RNG_RETRIES = 8192;

TEST(CalibrationStructsTest, testIntrinsicsStruct)
{
    // Here we test interoperability of intrinsics struct
    // and returned projection matrix
    corecvs::CameraIntrinsics intrinsics(1.0, 2.0, 3.0, 4.0, 5.0);
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
    corecvs::CameraIntrinsics intrinsics(1.0, 2.0, 3.0, 4.0, 5.0);
    corecvs::Camera_ camera(
            intrinsics, 
            LocationData(
                corecvs::Vector3dd(6.0, 7.0, 8.0),
                corecvs::Quaternion(0.5, 0.5, 0.5, 0.5)));
    auto M = camera.getKMatrix();

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

TEST(CalibrationStructsTest, testPhotostationStruct)
{
    // Here we test interoperability of photostation struct
    // and returned projection matrix
    corecvs::CameraIntrinsics intrinsics(1.0, 2.0, 3.0, 4.0, 5.0);
    corecvs::Camera_ camera(
            intrinsics, 
            corecvs::LocationData(
                corecvs::Vector3dd(6.0, 7.0, 8.0),
                corecvs::Quaternion(0.5, 0.5, 0.5, 0.5)));
    corecvs::Photostation ps;
    ps.location = corecvs::LocationData(
            corecvs::Vector3dd(9.0, 10.0, 11.0),
            corecvs::Quaternion(-1.0, 2.0, 3.0, -4.0).normalised());
    ps.cameras = { camera };

    corecvs::Matrix44 M[] = {
        ps.getKMatrix(0),
        ps.getRawCamera(0).getKMatrix()
    };
    auto C = ps.getRawCamera(0);

    std::mt19937 rng(DEFAULT_SEED);
    std::uniform_real_distribution<double> unif(-1e3, 1e3);

    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        corecvs::FixedVector<double, 4> src1;
        corecvs::Vector3dd src2;
        
        for (int j = 0; j < 3; ++j)
            src1[j] =  src2[j] = unif(rng);

        src1[3] = 1.0;

        auto dst1t = M[0] * src1;
        auto dst2 = C.project(src2);
        auto dst3 = ps.project(src2, 0);
        auto dst4t = M[1] * src1;

        corecvs::Vector2dd dst1(dst1t[0] / dst1t[2], dst1t[1] / dst1t[2]);
        corecvs::Vector2dd dst4(dst4t[0] / dst4t[2], dst4t[1] / dst4t[2]);
        
        auto ref = dst3;
        corecvs::Vector2dd check[] = { dst1, dst2, dst4 };
        for (int k = 0; k < 3; ++k)
            for (int j = 0; j < 2; ++j)
                ASSERT_NEAR(check[k][j], ref[j], 1e-6);
    }
}

TEST(CalibrationStructsTest, testIntrinsicsStructisVisible)
{
    corecvs::CameraIntrinsics intrinsics(1.0, 1.0, 0.5, 0.5, 0.0, corecvs::Vector2dd(1.0, 1.0));
    ASSERT_TRUE (intrinsics.isVisible(corecvs::Vector3dd( 0.0,  0.0, 1.0)));
    ASSERT_FALSE(intrinsics.isVisible(corecvs::Vector3dd( 2.0,  0.0, 1.0)));
    ASSERT_FALSE(intrinsics.isVisible(corecvs::Vector3dd(-2.0,  0.0, 1.0)));
    ASSERT_FALSE(intrinsics.isVisible(corecvs::Vector3dd( 0.0, -2.0, 1.0)));
    ASSERT_FALSE(intrinsics.isVisible(corecvs::Vector3dd( 0.0,  2.0, 1.0)));
}
