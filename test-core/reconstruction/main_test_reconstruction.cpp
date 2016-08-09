#include "gtest/gtest.h"

#include "global.h"

#ifdef WITH_BLAS
#include "pnpSolver.h"
#include "relativeNonCentralP6PSolver.h"
#include "relativeNonCentralO3PSolver.h"
#include "calibrationCamera.h"
#include "calibrationPhotostation.h"
#include "affine.h"
#include "sceneAligner.h"
#include "fixtureScene.h"
#include "sceneGenerator.h"
#include "reconstructionFunctor.h"

#include <random>
#include <map>
#include <iomanip>

using namespace corecvs;

const int DEFAULT_SEED = 777;
const int RNG_RETRIES = 8192;

TEST(Reconstruction, symbolicCheckRD)
{
    corecvs::SceneGeneratorParams params( 10.0, 500.0, 0.0, 0.9, 1.0, 0.75, 3, 80, 80, 10.0, 100.0, 10.0, M_PI / 3, M_PI, true);
    corecvs::SceneGenerator sg(params);
    sg.generateScene();
    std::unique_ptr<corecvs::ReconstructionFixtureScene> scene(sg.rfs);
    auto& pts = scene->featurePoints();
    for (auto& in: scene->initializationData)
        in.second.positioningAccuracy = corecvs::Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9);
    for (auto& pt: pts)
        scene->trackedFeatures.push_back(pt);
    scene->placedFixtures = scene->fixtures();
    //corecvs::ReconstructionFunctor rf(scene.get(), scene->placedFixtures, ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::REPROJECTION, corecvs::ReconstructionFunctorOptimizationType::FOCALS | corecvs::ReconstructionFunctorOptimizationType::PRINCIPALS | corecvs::ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS | corecvs::ReconstructionFunctorOptimizationType::NON_DEGENERATE_ORIENTATIONS | corecvs::ReconstructionFunctorOptimizationType::DEGENERATE_TRANSLATIONS | corecvs::ReconstructionFunctorOptimizationType::NON_DEGENERATE_TRANSLATIONS, true, 1.0);
    corecvs::ReconstructionFunctor rf(scene.get(), scene->placedFixtures, ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::RAY_DIFF,
               corecvs::ReconstructionFunctorOptimizationType::POINTS |
            corecvs::ReconstructionFunctorOptimizationType::PRINCIPALS |
            corecvs::ReconstructionFunctorOptimizationType::FOCALS |
            corecvs::ReconstructionFunctorOptimizationType::NON_DEGENERATE_TRANSLATIONS |
            corecvs::ReconstructionFunctorOptimizationType::DEGENERATE_TRANSLATIONS |
            corecvs::ReconstructionFunctorOptimizationType::NON_DEGENERATE_ORIENTATIONS |
            corecvs::ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS
//            corecvs::ReconstructionFunctorOptimizationType::TUNE_GPS
//
            ,  true, true, 1.0);
    std::vector<double> in(rf.getInputNum()), out(rf.getOutputNum());
    rf.writeParams(&in[0]);

    auto m = rf.SparseFunctionArgs::getNativeJacobian(&in[0], 1e-7);
    auto m2= rf.getNativeJacobian(&in[0], 1e-7);

#if 0
    std::cout << "=================================" << std::endl;
    std::cout << "M2 spyplot" << std::endl;
    m2.spyPlot();
    std::cout << "=================================" << std::endl;
    std::cout << "M spyplot" << std::endl;
    m.spyPlot();
    std::cout << "=================================" << std::endl;
#endif
#if 0
    for (int i = 0; i < m.h; ++i)
    {
        for (int j = 0; j < m.w; ++j)
        {
            double numeric = m.a(i, j),
                   symbolic= m2.a(i, j);
            if ((std::abs(numeric) != 0.0) ^ (std::abs(symbolic) != 0.0))
            {
                std::cout << "0";
                continue;
            }
            if (numeric * symbolic < 0.0)
            {
                std::cout << "-";
//        std::cout << "[" << numeric << " " << symbolic << "]";
                continue;
            }
            if (numeric * symbolic == 0.0)
            {
                CORE_ASSERT_TRUE_S(numeric == 0.0 && symbolic == 0.0);
                std::cout << " ";
//        std::cout << "[" << numeric << " " << symbolic << "]";
                continue;
            }
            double diff = numeric - symbolic;
            if (std::abs(diff) / std::abs(numeric) < 1e-2)
                std::cout << "*";
            else
                std::cout << "+";
//    std::cout << "[" << numeric << " " << symbolic << "]";
        }
        std::cout << std::endl;
    }
#endif
    std::cout << ((corecvs::Matrix)m).frobeniusNorm() << " vs " << ((corecvs::Matrix)m2).frobeniusNorm() << std::endl;
    corecvs::Matrix M = (corecvs::Matrix)(m - m2);
    std::cout << std::sqrt(M.frobeniusNorm() / m.nnz()) << std::endl;
    ASSERT_LE(std::sqrt(M.frobeniusNorm() / m.nnz()), 1e-5);
}


TEST(Reconstruction, symbolicCheckRE)
{
    corecvs::SceneGeneratorParams params( 10.0, 500.0, 0.0, 0.9, 1.0, 0.75, 3, 80, 80, 10.0, 100.0, 10.0, M_PI / 3, M_PI, true);
    corecvs::SceneGenerator sg(params);
    sg.generateScene();
    std::unique_ptr<corecvs::ReconstructionFixtureScene> scene(sg.rfs);
    auto& pts = scene->featurePoints();
    for (auto& in: scene->initializationData)
        in.second.positioningAccuracy = corecvs::Matrix33(1, 2, 3, 4, 5, 6, 7, 8, 9);
    for (auto& pt: pts)
        scene->trackedFeatures.push_back(pt);
    scene->placedFixtures = scene->fixtures();
    //corecvs::ReconstructionFunctor rf(scene.get(), scene->placedFixtures, ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::REPROJECTION, corecvs::ReconstructionFunctorOptimizationType::FOCALS | corecvs::ReconstructionFunctorOptimizationType::PRINCIPALS | corecvs::ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS | corecvs::ReconstructionFunctorOptimizationType::NON_DEGENERATE_ORIENTATIONS | corecvs::ReconstructionFunctorOptimizationType::DEGENERATE_TRANSLATIONS | corecvs::ReconstructionFunctorOptimizationType::NON_DEGENERATE_TRANSLATIONS, true, 1.0);
    corecvs::ReconstructionFunctor rf(scene.get(), scene->placedFixtures, ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::REPROJECTION,
               corecvs::ReconstructionFunctorOptimizationType::POINTS |
            corecvs::ReconstructionFunctorOptimizationType::PRINCIPALS |
            corecvs::ReconstructionFunctorOptimizationType::FOCALS |
            corecvs::ReconstructionFunctorOptimizationType::NON_DEGENERATE_TRANSLATIONS |
            corecvs::ReconstructionFunctorOptimizationType::DEGENERATE_TRANSLATIONS |
            corecvs::ReconstructionFunctorOptimizationType::NON_DEGENERATE_ORIENTATIONS |
            corecvs::ReconstructionFunctorOptimizationType::DEGENERATE_ORIENTATIONS
//            corecvs::ReconstructionFunctorOptimizationType::TUNE_GPS
//
            ,  true, true, 1.0);
    std::vector<double> in(rf.getInputNum()), out(rf.getOutputNum());
    rf.writeParams(&in[0]);

    auto m = rf.SparseFunctionArgs::getNativeJacobian(&in[0], 1e-7);
    auto m2= rf.getNativeJacobian(&in[0], 1e-7);

#if 0
    std::cout << "=================================" << std::endl;
    std::cout << "M2 spyplot" << std::endl;
    m2.spyPlot();
    std::cout << "=================================" << std::endl;
    std::cout << "M spyplot" << std::endl;
    m.spyPlot();
    std::cout << "=================================" << std::endl;
#endif
#if 0
    for (int i = 0; i < m.h; ++i)
    {
        for (int j = 0; j < m.w; ++j)
        {
            double numeric = m.a(i, j),
                   symbolic= m2.a(i, j);
            if ((std::abs(numeric) != 0.0) ^ (std::abs(symbolic) != 0.0))
            {
                std::cout << "0";
                continue;
            }
            if (numeric * symbolic < 0.0)
            {
                std::cout << "-";
//        std::cout << "[" << numeric << " " << symbolic << "]";
                continue;
            }
            if (numeric * symbolic == 0.0)
            {
                CORE_ASSERT_TRUE_S(numeric == 0.0 && symbolic == 0.0);
                std::cout << " ";
//        std::cout << "[" << numeric << " " << symbolic << "]";
                continue;
            }
            double diff = numeric - symbolic;
            if (std::abs(diff) / std::abs(numeric) < 1e-2)
                std::cout << "*";
            else
                std::cout << "+";
//    std::cout << "[" << numeric << " " << symbolic << "]";
        }
        std::cout << std::endl;
    }
#endif
    std::cout << ((corecvs::Matrix)m).frobeniusNorm() << " vs " << ((corecvs::Matrix)m2).frobeniusNorm() << std::endl;
    corecvs::Matrix M = (corecvs::Matrix)(m - m2);
    std::cout << std::sqrt(M.frobeniusNorm() / m.nnz()) << std::endl;
    ASSERT_LE(std::sqrt(M.frobeniusNorm() / m.nnz()), 1e-5);
}

TEST(Reconstruction, basicO3P)
{
    double angleThreshold = 1.0;
    std::mt19937 rng = std::mt19937(std::random_device()());
    std::uniform_real_distribution<double> runif(-1, 1);
    std::vector<corecvs::Vector3dd> originRef(3), originQuery(3), ptRef(3), ptQuery(3);
    corecvs::Vector3dd shift;
    corecvs::Quaternion rotor;

    std::cout << std::setprecision(22);
    int failed = 0;
    for (int ii = 0; ii < RNG_RETRIES; ++ii)
    {
    for (int i = 0; i < 3; ++i)
    {
        shift[i] = runif(rng) * 10.0;
        rotor[i] = runif(rng);

        for (int j = 0; j < 3; ++j)
        {
            originRef[i][j] = runif(rng) * 0.1;
            originQuery[i][j] = runif(rng) * 0.1;
            ptRef[i][j] = runif(rng) * 1000;
            rotor[3] = runif(rng);
        }
    }
//    shift = corecvs::Vector3dd(1, 1, 1);
    rotor.normalise();
    for (int i = 0; i < 3; ++i)
        ptQuery[i] = rotor.conjugated() * (ptRef[i] - shift);

    std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> l, r;
    for (int i = 0; i < 3; ++i)
    {
        l.emplace_back(corecvs::Ray3d(ptRef[i] - originRef[i], originRef[i]).normalised().pluckerize());
        r.emplace_back(corecvs::Ray3d(ptQuery[i] - originQuery[i], originQuery[i]).normalised().pluckerize());
    }
#if 0
        std::cout << "W = [" << rotor[0] << "; " << rotor[1] << "; " << rotor[2] << "; " << rotor[3] << "];" << std::endl;
        std::cout << "origins_ref = [";
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                std::cout << originRef[j][i] << (j == 2 ? "; " : ", ");
        std::cout << "];" << std::endl;
        std::cout << "origins_query = [";
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                std::cout << originQuery[j][i] << (j == 2 ? "; " : ", ");
        std::cout << "];" << std::endl;
        std::cout << "points_ref = [";
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                std::cout << ptRef[j][i] << (j == 2 ? "; " : ", ");
        std::cout << "];" << std::endl;
        std::cout << "points_query = [";
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                std::cout << ptQuery[j][i] << (j == 2 ? "; " : ", ");
        std::cout << "];" << std::endl;
        std::cout << "T=[" << shift[0] << "; " << shift[1] << "; " << shift[2] << "];" << std::endl;
#endif
    auto h = corecvs::RelativeNonCentralO3PSolver::SolveRelativeNonCentralO3P(l, r, shift);
    double best = 1e10;
    for (auto& hh: h)
    {
        double foo = std::acos(std::abs((hh.rotor^rotor.conjugated())[3]))*360.0/M_PI;
        if (foo < best)
            best = foo;
    }
        if (best > angleThreshold)
            failed++;
    }
    std::cout << ((double)failed) / RNG_RETRIES << std::endl;
    ASSERT_LE(failed, 0.1 * RNG_RETRIES);
}


TEST(Reconstruction, nonCentralO3P)
{
    std::mt19937 rng(DEFAULT_SEED);
    std::uniform_real_distribution<double> runif(-1, 1);
    std::map<int, int> boo;
    FixtureScene scene;
    auto fr = scene.createCameraFixture(),
         fq = scene.createCameraFixture(),
         fqq= scene.createCameraFixture();
    fr->location.shift = corecvs::Vector3dd(0.0, 0.0, 0.0);
    fr->location.rotor = corecvs::Quaternion(0.0, 0.0, 0.0, 1.0);
    fqq->location.shift = corecvs::Vector3dd(0, 0, 0);
    fqq->location.rotor = corecvs::Quaternion(0.0, 0.0, 0.0, 1.0);

    for (int i = 0; i < 6; ++i)
    {
        auto c = scene.createCamera();
        scene.addCameraToFixture(c, fq);
        scene.addCameraToFixture(c, fr);
        scene.addCameraToFixture(c, fqq);

        c->extrinsics.orientation = corecvs::Quaternion(0, sin(M_PI / 6.0 * i), 0, cos(M_PI / 6.0 * i));
        c->extrinsics.position = corecvs::Vector3dd(sin(M_PI / 3.0 * i), 0, cos(M_PI / 3.0 * i));
        c->intrinsics = PinholeCameraIntrinsics(17.0, 17.0, 100.0, 100.0, 0.0, Vector2dd(200, 200), Vector2dd(200, 200));
    }

    int failed = 0;
    double angularThreshold = 1.0;

    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        auto &loc = fq->location;
        for (int i = 0; i < 3; ++i)
        {
            loc.shift[i] = runif(rng)*10;
            loc.rotor[i] = runif(rng);
        }
        loc.rotor.normalise();
        std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> LRays, RRays;
        for (int rdy = 0; rdy < 3; )
        {
            corecvs::Vector3dd pt(runif(rng), runif(rng), runif(rng));
            pt *= 2000;
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 6; ++k)
                {
                    if (!fr->isVisible(pt, j) || !fq->isVisible(pt, k))
                        continue;
                    LRays.emplace_back(fr->rayFromPixel(fr->cameras[j], fr->project(pt, j)).normalised().pluckerize());
                    RRays.emplace_back(fqq->rayFromPixel(fqq->cameras[k], fq->project(pt, k)).normalised().pluckerize());
                    rdy++;
                    goto lexit;
                }
            lexit:;
        }

        auto h = corecvs::RelativeNonCentralO3PSolver::SolveRelativeNonCentralO3P(LRays, RRays, fq->location.shift);

        ASSERT_GE(h.size(), 1);
        std::sort(h.begin(), h.end(), [&](const corecvs::Affine3DQ &a, const corecvs::Affine3DQ &b)
                {
                    auto diffA = a.rotor ^ fq->location.rotor.conjugated(),
                         diffB = b.rotor ^ fq->location.rotor.conjugated();
                    return    (1.0-std::abs(diffA[3])) < (1.0-std::abs(diffB[3]));
                });
        auto diffBest = h[0].rotor ^ fq->location.rotor.conjugated();
        double angBest = acos(diffBest[3])*360.0/M_PI;
        angBest = angBest > 180.0 ? 360.0 - angBest : angBest;
        boo[angBest]++;
        if (angBest > angularThreshold)
            failed++;
    }
    for (auto& d: boo)
        std::cout << d.first << " : " << d.second << std::endl;
    ASSERT_LE(failed, 0.2 * RNG_RETRIES);
}

TEST(Reconstruction, alignerPoseFromVectors)
{
    std::mt19937 rng(DEFAULT_SEED);
    std::uniform_real_distribution<double> runif(-1e3, 1e3);
    int failureCntr = 0;

    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        corecvs::Vector3dd A, B, qA, qB;
        corecvs::Quaternion Q;

        for (int j = 0; j < 3; ++j)
        {
            A[j] = runif(rng);
            B[j] = runif(rng);
            Q[j] = runif(rng);
        }
        Q[3] = runif(rng);
        A.normalise();
        B.normalise();
        Q.normalise();
        qA = Q * A;
        qB = Q * B;

        auto Qe = SceneAligner::EstimateOrientationTransformation(qA, qB, A, B);
        Qe = Qe ^ Q.conjugated();
        if (Qe[3] < 0.0) Qe = -Qe;
        bool failed = false;
        for (int j = 0; j < 3; ++j)
            if (std::abs(Qe[j]) > 1e-6)
                failed |= true;
        if (std::abs(1.0 - Qe[3]) > 1e-6)
            failed |= true;
        if (failed) failureCntr++;
    }
    std::cout << ((double)failureCntr) / RNG_RETRIES * 100.0 << "% failures" << std::endl;
    ASSERT_LE(failureCntr, 1e-3 * RNG_RETRIES);
}

TEST(Reconstruction, nonCentralRelative6P)
{
    double angleThreshold = 0.5;
    corecvs::CameraModel cam(PinholeCameraIntrinsics(100.0, 100.0, 100.0, 100.0, 0.0, Vector2dd(800, 800), Vector2dd(800, 800)));
    corecvs::Photostation ps;
    for (int i = 0; i < 6; ++i)
    {
        ps.cameras.push_back(cam);
        ps.cameras[i].extrinsics.orientation = corecvs::Quaternion(0, sin(M_PI / 6.0 * i), 0, cos(M_PI / 6.0 * i));
        ps.cameras[i].extrinsics.position = corecvs::Vector3dd(sin(M_PI / 3.0 * i), 0, cos(M_PI / 3.0 * i));

    }
    ps.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    ps.location.shift = corecvs::Vector3dd(0, 0, 0);

    corecvs::Photostation ps1 = ps;
    corecvs::Photostation ps2 = ps;
    ps1.location.shift = corecvs::Vector3dd(0, 0, 0);
    ps1.location.rotor = corecvs::Quaternion(0, 0, 0, 1);

    ps2.location.shift = corecvs::Vector3dd(166, 0, 0);
    ps2.location.rotor = corecvs::Quaternion(0, sin(0.25*M_PI/180.0), 0, cos(0.25*M_PI/180.0)).normalised();
    std::cout << ps2.location.rotor << std::endl;
    std::cout << ps2.location.shift << std::endl;
    std::cout << ps1.location.rotor << std::endl;
    std::cout << ps1.location.shift << std::endl;

    std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> LR, RR;

    std::mt19937 rng(DEFAULT_SEED);
    std::uniform_real_distribution<double> runif(-1e3, 1e3);
    int used[6] = {0};
    for (int i = 0; LR.size() < 6; ++i)
    {
        corecvs::Vector3dd pt(runif(rng), runif(rng), runif(rng));
        if (!ps1.isVisible(pt) || !ps2.isVisible(pt))
            continue;
        bool found = false;
        for (int j = 0; j < 6 && !found; ++j)
        {
            for (int k = 0; k < 6 && !found; ++k)
            {
                if (ps1.isVisible(pt, j) && ps2.isVisible(pt, k) && !used[j])
                {
                    used[j] = 1;
                    found = true;
                    auto c1 = ps1.getRawCamera(j);
                    auto c2 = ps2.getRawCamera(k);
                    auto p1 = c1.project(pt);
                    auto p2 = c2.project(pt);
                    auto r1 = c1.rayFromPixel(p1);
                    auto r2 = ps.getRawCamera(k).rayFromPixel(p2);
                    LR.emplace_back(r1.pluckerize());
                    RR.emplace_back(r2.pluckerize());
                }
            }
        }
    }
    auto ans = corecvs::RelativeNonCentralP6PSolver::SolveRelativeNonCentralP6P(LR, RR);
    std::sort(ans.begin(), ans.end(), [=](const corecvs::Affine3DQ &a, const corecvs::Affine3DQ &b)
            {
                auto q1 = a.rotor ^ ps2.location.rotor;
                auto q2 = b.rotor ^ ps2.location.rotor;
                double a1 = std::acos(q1[3]) * 2.0;
                double a2 = std::acos(q2[3]) * 2.0;
                a1 = std::abs(a1 > M_PI ? 2.0 * M_PI - a1 : a1);
                a2 = std::abs(a2 > M_PI ? 2.0 * M_PI - a2 : a2);
                return a1 > a2;
            });
    int closeCnt = 0;
    for (auto& hypo: ans)
    {
        std::cout << hypo.shift << " " << hypo.rotor << std::endl;
        double diff = std::acos((hypo.rotor.conjugated() ^ ps2.location.rotor)[3]) * 2.0;
        diff = std::min(std::abs(diff), std::abs(2*M_PI - diff)) * 180.0 / M_PI;
        if (hypo.shift.angleTo(ps2.location.shift) * 180.0 / M_PI < angleThreshold * !(ps2.location.shift) && diff < angleThreshold)
            closeCnt++;
    }
    ASSERT_TRUE(closeCnt > 0);
}

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
    int N = (int)points.size();

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

        double minDiffPos   = 1e10;
        //double minDiffAngle = 1e100;

        for (auto &r: res)
        {
            double diffPos = !(r.shift - pos);
            if (diffPos < minDiffPos)
            {
                minDiffPos = diffPos;
            }
        }
        // 1mm accuracy
        if (minDiffPos > 1e-3)
            cntInValid++;
    }
    std::cout << "Invalid: " << (((double)cntInValid)/RNG_RETRIES) << std::endl;
    ASSERT_LE(cntInValid, 0.01 * RNG_RETRIES);
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

        double minDiffPos   = 1e10;
        double minDiffAngle = 1e10;

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
    ASSERT_LE(cntInValid, 0.01 * RNG_RETRIES);
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

        double minDiffPos   = 1e10;
        double minDiffAngle = 1e10;

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
    ASSERT_LE(cntInValid, 0.01 * RNG_RETRIES);
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

        double minDiffPos   = 1e10;
        double minDiffAngle = 1e10;

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
    ASSERT_LE(cntInValid, 0.01 * RNG_RETRIES);
}
#endif // WITH_BLAS
