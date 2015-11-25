#include "gtest/gtest.h"

#include "global.h"

#include "pnpSolver.h"
#include "calibrationCamera.h"
#include "calibrationPhotostation.h"
#include "affine.h"

#include <random>

using namespace std;
using namespace corecvs;

const int DEFAULT_SEED = 777;
const int RNG_RETRIES = 1024;

TEST(Reconstruction, testNonCentralMulticamera)
{
    double tolerance = 1e-2;
    std::vector<corecvs::Vector3dd> points = {
        corecvs::Vector3dd( 100.0,   0.0,   0.0),
        corecvs::Vector3dd(-100.0,   0.0,   0.0),
        corecvs::Vector3dd(   0.0, 100.0,   0.0),
        corecvs::Vector3dd(   0.0,-100.0,   0.0),
        corecvs::Vector3dd(   0.0,   0.0, 100.0),
        corecvs::Vector3dd(   0.0,   0.0,-100.0),
        corecvs::Vector3dd( 100.0, 100.0,   0.0),
        corecvs::Vector3dd(-100.0, 100.0,   0.0),
        corecvs::Vector3dd( 100.0,-100.0,   0.0),
        corecvs::Vector3dd(-100.0,-100.0,   0.0),
        corecvs::Vector3dd(   0.0, 100.0, 100.0),
        corecvs::Vector3dd(   0.0, 100.0,-100.0),
        corecvs::Vector3dd(   0.0,-100.0, 100.0),
        corecvs::Vector3dd(   0.0,-100.0,-100.0),
        corecvs::Vector3dd( 100.0,-100.0,   0.0),
        corecvs::Vector3dd(-100.0, 100.0,   0.0),
        corecvs::Vector3dd( 100.0,-100.0,   0.0),
        corecvs::Vector3dd(-100.0,-100.0,   0.0)
    };
    int N = points.size();

    corecvs::CameraModel cam(PinholeCameraIntrinsics(100.0, 100.0, 100.0, 100.0, 0.0, Vector2dd(800, 800), Vector2dd(800, 800)));
    corecvs::Photostation ps;
    ps.cameras.push_back(cam);
    ps.cameras.push_back(cam);
    ps.cameras.push_back(cam);
    ps.cameras.push_back(cam);

    ps.cameras[0].extrinsics.position = corecvs::Vector3dd( 5, 0, 0);
    ps.cameras[1].extrinsics.position = corecvs::Vector3dd(-5, 0, 0);
    ps.cameras[2].extrinsics.position = corecvs::Vector3dd( 0, 5, 0);
    ps.cameras[3].extrinsics.position = corecvs::Vector3dd( 0,-5, 0);
    ps.cameras[0].extrinsics.orientation = corecvs::Quaternion::FromMatrix(corecvs::Matrix33(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0));
    ps.cameras[1].extrinsics.orientation = corecvs::Quaternion::FromMatrix(corecvs::Matrix33(0.0,  1.0, 0.0, 0.0, 0.0, -1.0,-1.0, 0.0, 0.0));
    ps.cameras[2].extrinsics.orientation = corecvs::Quaternion::FromMatrix(corecvs::Matrix33(1.0,  0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0));
    ps.cameras[3].extrinsics.orientation = corecvs::Quaternion::FromMatrix(corecvs::Matrix33(-1.0,  0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -1.0, 0.0));
    auto psr = ps;

    ps.location.shift = corecvs::Vector3dd(5.0, 6.0, 7.0);
    ps.location.rotor = corecvs::Quaternion(1.0, 2.0, 3.0, 4.0).normalised();

    ps.location.rotor.printAxisAndAngle();

    std::vector<corecvs::Vector3dd> pts, dirs, offsets;
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (ps.isVisible(points[i], j))
            {
                auto cam = ps.getRawCamera(j);
                auto proj = cam.project(points[i]);
                auto dir = psr.getRawCamera(j).rayFromPixel(proj).a.normalised();
                auto pt = points[i];
                auto diff = pt - cam.rayFromPixel(proj).p;
                pts.push_back(pt);
                dirs.push_back(dir);
                offsets.push_back(psr.getRawCamera(j).rayFromPixel(proj).p);
            }
        }
    }
    
    std::cout << std::endl << std::endl;
    auto res = corecvs::PNPSolver::solvePNP(offsets, dirs, pts);
    int nearEnough = 0;
    for (auto& r: res)
    {
        double distDiff = !(r.shift - ps.location.shift);
        double angleDiff = std::abs((r.rotor ^ ps.location.rotor)[3]) - 1.0;
        angleDiff = std::abs(angleDiff) - 1.0;
        if (angleDiff * angleDiff + distDiff * distDiff < tolerance)
            nearEnough++;
    }
    ASSERT_TRUE(nearEnough > 0);
}

TEST(Reconstruction, testP3P)
{
    std::mt19937 rng(DEFAULT_SEED);
    // 100x100m cube
    std::uniform_real_distribution<double> runif(-1e2, 1e2);
    int NPT = 3;
    int cntInValid = 0;
    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        std::vector<corecvs::Vector3dd> pts, dirs;
        corecvs::Vector3dd pos(runif(rng), runif(rng), runif(rng));
        for (int j = 0; j < NPT; ++j)
        {
            pts.emplace_back(runif(rng), runif(rng), runif(rng));
            dirs.emplace_back((*pts.rbegin() - pos).normalised());
        }

        auto res = corecvs::PNPSolver::solvePNP(dirs, pts);

        double minDiffPos   = 1e100;
        double minDiffAngle = 1e100;

        for (auto &r: res)
        {
            double diffPos = !(r.shift - pos);
            double diffAng = r.rotor.getAngle();
            if (diffAng > M_PI)
                diffAng = 2.0 * M_PI - diffAng;
            if (diffPos < minDiffPos)
            {
                minDiffPos = diffPos;
                minDiffAngle = diffAng;
            }
        }
        // 1mm accuracy
        if (minDiffPos > 1e-3)
            cntInValid++;
    }
    std::cout << "Invalid: " << (((double)cntInValid)/RNG_RETRIES) << std::endl;
    ASSERT_LE(cntInValid, 0.001 * RNG_RETRIES);
}

TEST(Reconstruction, testP4P)
{
    std::mt19937 rng(DEFAULT_SEED);
    // 100x100m cube
    std::uniform_real_distribution<double> runif(-1e2, 1e2);
    int NPT = 4;
    int cntInValid = 0;
    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        std::vector<corecvs::Vector3dd> pts, dirs;
        corecvs::Vector3dd pos(runif(rng), runif(rng), runif(rng));
        for (int j = 0; j < NPT; ++j)
        {
            pts.emplace_back(runif(rng), runif(rng), runif(rng));
            dirs.emplace_back((*pts.rbegin() - pos).normalised());
        }

        auto res = corecvs::PNPSolver::solvePNP(dirs, pts);

        double minDiffPos   = 1e100;
        double minDiffAngle = 1e100;

        for (auto &r: res)
        {
            double diffPos = !(r.shift - pos);
            double diffAng = r.rotor.getAngle();
            if (diffAng > M_PI)
                diffAng = 2.0 * M_PI - diffAng;
            if (diffPos < minDiffPos && diffAng < minDiffAngle)
            {
                minDiffPos = diffPos;
                minDiffAngle = diffAng;
            }
        }
        // 1mm accuracy
        if (minDiffPos > 1e-3)
            cntInValid++;
    }
    std::cout << "Invalid: " << (((double)cntInValid)/RNG_RETRIES) << std::endl;
    ASSERT_LE(cntInValid, 0.001 * RNG_RETRIES);
}

TEST(Reconstruction, testP6P)
{
    std::mt19937 rng(DEFAULT_SEED);
    // 100x100m cube
    std::uniform_real_distribution<double> runif(-1e2, 1e2);
    int NPT = 6;
    int manySols = 0;
    int cntInValid = 0;
    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        std::vector<corecvs::Vector3dd> pts, dirs;
        corecvs::Vector3dd pos(runif(rng), runif(rng), runif(rng));
        for (int j = 0; j < NPT; ++j)
        {
            pts.emplace_back(runif(rng), runif(rng), runif(rng));
            dirs.emplace_back((*pts.rbegin() - pos).normalised());
        }

        auto res = corecvs::PNPSolver::solvePNP(dirs, pts);

        double minDiffPos   = 1e100;
        double minDiffAngle = 1e100;

        for (auto &r: res)
        {
            double diffPos = !(r.shift - pos);
            double diffAng = r.rotor.getAngle();
            if (diffAng > M_PI)
                diffAng = 2.0 * M_PI - diffAng;
            if (diffPos < minDiffPos && diffAng < minDiffAngle)
            {
                minDiffPos = diffPos;
                minDiffAngle = diffAng;
            }
        }
        if (minDiffPos > 1e-3)
            cntInValid++;
    }
    std::cout << "Invalid: " << (((double)cntInValid)/RNG_RETRIES) << std::endl;
    ASSERT_LE(cntInValid, 0.001 * RNG_RETRIES);
}

TEST(Reconstruction, testPNP)
{
    std::mt19937 rng(DEFAULT_SEED);
    // 100x100m cube
    std::uniform_real_distribution<double> runif(-1e2, 1e2);
    int cntInValid = 0;
    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        int NPT = 3 + (rng() % 100);
        std::vector<corecvs::Vector3dd> pts, dirs;
        corecvs::Vector3dd pos(runif(rng), runif(rng), runif(rng));
        for (int j = 0; j < NPT; ++j)
        {
            pts.emplace_back(runif(rng), runif(rng), runif(rng));
            dirs.emplace_back((*pts.rbegin() - pos).normalised());
        }

        auto res = corecvs::PNPSolver::solvePNP(dirs, pts);

        double minDiffPos   = 1e100;
        double minDiffAngle = 1e100;

        for (auto &r: res)
        {
            double diffPos = !(r.shift - pos);
            double diffAng = r.rotor.getAngle();
            if (diffAng > M_PI)
                diffAng = 2.0 * M_PI - diffAng;
            if (diffPos < minDiffPos && diffAng < minDiffAngle)
            {
                minDiffPos = diffPos;
                minDiffAngle = diffAng;
            }
        }
        if (minDiffPos > 1e-3)
            cntInValid++;
    }
    std::cout << "Invalid: " << (((double)cntInValid)/RNG_RETRIES) << std::endl;
    ASSERT_LE(cntInValid, 0.001 * RNG_RETRIES);
}
