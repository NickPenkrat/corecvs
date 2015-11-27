#include "gtest/gtest.h"

#include "global.h"

#include "pnpSolver.h"

#include <random>

using namespace std;
using namespace corecvs;

const int DEFAULT_SEED = 777;
const int RNG_RETRIES = 1024;

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
    //int manySols = 0;
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
