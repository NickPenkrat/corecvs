#ifndef ASSERTS
#define ASSERTS
#endif

#include "gtest/gtest.h"

#include "core/utils/global.h"
#include "core/math/vector/vector4d.h"

#include "core/cameracalibration/cameraModel.h"
#include "core/cameracalibration/calibrationLocation.h"

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
                Vector2dd p1 = camera1.project(pt);
                Vector2dd p2 = camera2.project(pt);
                Vector3dd L(p1.x(), p1.y(), 1.0);
                Vector3dd R(p2.x(), p2.y(), 1.0);

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
        auto K1 = camera1.getPinhole()->getKMatrix33().inv();
        auto K2 = camera2.getPinhole()->getKMatrix33().inv();
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
    vector<EulerAngles> input;
    input.push_back({45.0, 10.0, 2.0});
    input.push_back({   0,   90,   0});

    for (size_t testId = 0; testId < input.size(); testId++)
    {
        cout << "Test :" << testId << endl;

        {
            EulerAngles &test = input[testId];
            CameraLocationAngles angles = CameraLocationAngles::FromAnglesDeg(test.alpha, test.beta, test.gamma);
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

            CORE_ASSERT_TRUE_P(q.notTooFar(q1, 1e-6), ("Test failed %" PRISIZE_T " \n", testId));
            CORE_ASSERT_TRUE_P(q.notTooFar(qR, 1e-6), ("Test failed %" PRISIZE_T " \n", testId));
        }
    }
}

#if 0
TEST(CalibrationStructsTest, testStructConversionWorld)
{
    Matrix33   m = Matrix33( 0, -1,  0,
                             0,  0, -1,
                             1,  0,  0
                   );
    Quaternion q = Quaternion::FromMatrix(m);
    Affine3DQ  a = Affine3DQ(q);


    cout << "Matrix: "     << m << endl;
    cout << "Quaternion: " << q << endl;
    cout << "Affine3DQ: "  << a << endl;

    WorldLocationAngles wl = WorldLocationAngles::FromQuaternion(q);
    cout << "WorldLocationAngles: \n" << wl << endl;

    CameraLocationAngles cl = CameraLocationAngles::FromQuaternion(q);
    cout << "CameraLocationAngles: \n" << wl << endl;

}
#endif


TEST(CalibrationStructsTest, testFrustrumMatrix)
{
    CameraModel model;
    model.intrinsics.reset(new PinholeCameraIntrinsics(Vector2dd(400,400), degToRad(60)));
    model.setLocation(Affine3DQ());

    Matrix44 mF = model.getPinhole()->getFrustumMatrix(1.0, 1000.0);
    cout << "Frustrum Matrix" << endl;
    cout << mF;

    {
        Vector3dd x1(0, 0, 10);
        cout << x1 << " -> " << mF * x1 << endl;
    }

    cout << "Frustrum Matrix old" << endl;
    cout << Matrix44::Frustum(degToRad(60), 1.0, 1.0, 1000.0) << endl;
}
