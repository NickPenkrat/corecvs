#include "reconstructionInitializer.h"

#include "relativeNonCentralRansacSolver.h"
#include "absoluteNonCentralRansacSolver.h"
#include "reconstructionStructs.h"
#include "log.h"

#ifdef WITH_TBB
#include <tbb/tbb.h>
#endif

bool corecvs::ReconstructionInitializer::initialize()
{
    if (scene->state != ReconstructionState::MATCHED)
        return false;
    CORE_ASSERT_TRUE_S(scene->placingQueue.size() >= 2);
    std::unordered_map<FixtureInitializationType, int> cnt;
    for (size_t i = 0; i < std::min((size_t)3, scene->placingQueue.size()); ++i)
        cnt[scene->initializationData[scene->placingQueue[i]].initializationType]++;

    // Gives 6-DoF initialization + 3-view cloud (1)
    if (cnt[FixtureInitializationType::GPS] == 3)
        return initGPS();
    // Gives 6-DoF initialization + 2-view cloud (16)
    if (cnt[FixtureInitializationType::FIXED] >= 1 || cnt[FixtureInitializationType::STATIC] >= 1)
    {
        return cnt[FixtureInitializationType::FIXED] > cnt[FixtureInitializationType::STATIC] ? initFIXED() : initSTATIC();
    }
    if (cnt[FixtureInitializationType::GPS] > 0)
    {
        // requires DoF estimation on the fly, NIY
        CORE_ASSERT_TRUE_S(false);
    }
    // Gives 0-DoF initialization + 2-view cloud
    return initNONE();
}


bool corecvs::ReconstructionInitializer::initGPS()
{
    L_INFO << "Starting feature filtering";
    std::vector<CameraFixture*> pss = {scene->placingQueue[0], scene->placingQueue[1], scene->placingQueue[2]};
    if (runEssentialFiltering)
        scene->filterEssentialRansac(pss, pss, essentialFilterParams);
    else
        scene->matchesCopy = scene->matches;

    L_INFO << "Estimating first pair orientation";
    estimateFirstPair();
    return true;
}

bool corecvs::ReconstructionInitializer::initNONE()
{
    CORE_ASSERT_TRUE_S(false);
    return true;
}

bool corecvs::ReconstructionInitializer::initSTATIC()
{
    L_INFO << "Initializing 3 pss";
    for (int i = 0; i < 3; ++i)
    {
        auto psApp = scene->placingQueue[i];
        corecvs::AbsoluteNonCentralRansacSolverParams params;
        params.reprojectionInlierThreshold = 16.0;
        std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> foo;
        auto& initPts = scene->initializationData[psApp].staticPoints;
        for (auto& ptr: initPts)
        {
            for (auto& obs: ptr->observations__)
            {
                if (obs.first.u != psApp)
                    continue;
                auto cam = obs.first.v;
                auto proj= obs.second.observation;
                auto ptw = ptr->position;
                foo.emplace_back(cam, proj, ptw, ptr, -1);
            }
            bool alreadyIn = false;
            for (auto& ptrS: scene->staticPoints)
                if (ptrS == ptr)
                {
                    alreadyIn = true;
                    break;
                }
            if (alreadyIn)
                continue;
            scene->staticPoints.push_back(ptr);
            ptr->reprojectedPosition = ptr->position;
        }
        AbsoluteNonCentralRansacSolver solver(psApp, foo, params);
        solver.cloudMatches = foo;
        solver.run();
        solver.runInliersRE();
        auto hypo = solver.getBestHypothesis();
        psApp->location = hypo;
        std::cout << "!!!!"  << psApp->name << hypo << "!!!!" << std::endl;
    }
//    dumpMesh("staticinit.ply");
    return true;
}

bool corecvs::ReconstructionInitializer::initFIXED()
{
    CORE_ASSERT_TRUE_S(false);
    return true;
}

void corecvs::ReconstructionInitializer::estimateFirstPair()
{
    auto A = scene->placingQueue[0],
         B = scene->placingQueue[1],
         C = scene->placingQueue[2];
#ifdef WITH_TBB
    tbb::task_group g;
    g.run([=]() { estimatePair(A, B); });
    g.run([=]() { estimatePair(A, C); });
    g.wait();
#else
    estimatePair(A, B);
    estimatePair(A, C);
#endif

    auto q = detectOrientationFirst(A, B, C);
    A->location.rotor = q.conjugated();
    A->location.shift = scene->initializationData[A].initData.shift;

    B->location.rotor = q.conjugated() ^ B->location.rotor;
    B->location.shift = scene->initializationData[B].initData.shift;

    C->location.rotor = q.conjugated() ^ C->location.rotor;
    C->location.shift = scene->initializationData[C].initData.shift;

    scene->matches = scene->matchesCopy;
}

corecvs::Quaternion corecvs::ReconstructionInitializer::detectOrientationFirst(CameraFixture* psA, CameraFixture* psB, CameraFixture* psC)
{
    auto init = scene->initializationData;
    corecvs::Vector3dd e1 = init[psB].initData.shift - init[psA].initData.shift;
    corecvs::Vector3dd e2 = init[psC].initData.shift - init[psA].initData.shift;

    corecvs::Vector3dd o1 = psB->location.shift - psA->location.shift;
    corecvs::Vector3dd o2 = psC->location.shift - psA->location.shift;
    return TransformFrom2RayCorrespondence(o1, o2, e1, e2);
}

void corecvs::ReconstructionInitializer::estimatePair(CameraFixture *psA, CameraFixture *psB)
{
    auto matches = scene->getPhotostationMatches({psA}, psB);
    RelativeNonCentralRansacSolver::MatchContainer rm, mm;
    for (auto&t : matches)
    {
        if (std::get<4>(t) < b2bThreshold)
            rm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
        mm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
    }

    RelativeNonCentralRansacSolver solver(
            psB, rm, mm);
    solver.run();
    auto best = solver.getBestHypothesis();
    std::cout << psA->name << "::" << psB->name << " " << best.shift << " " << best.rotor << std::endl;
    best = solver.getBestHypothesis();
    std::cout << psA->name << "::" << psB->name << " " << best.shift << " " << best.rotor << std::endl;
    psB->location = best;
}

corecvs::Quaternion corecvs::ReconstructionInitializer::TransformFrom2RayCorrespondence(corecvs::Vector3dd o1, corecvs::Vector3dd o2, corecvs::Vector3dd e1, corecvs::Vector3dd e2)
{

    e1.normalise();
    e2.normalise();
    o1.normalise();
    o2.normalise();

    corecvs::Vector3dd e3 = e1 ^ e2;
    corecvs::Vector3dd o3 = o1 ^ o2;

    corecvs::Matrix A(9, 9);
    corecvs::Vector B(9);
    corecvs::Matrix33 RC;

    for (int i = 0; i < 3; ++i)
    {
        A.a(0, i) = A.a(1, i + 3) = A.a(2, i + 6) = e1[i];
        A.a(3, i) = A.a(4, i + 3) = A.a(5, i + 6) = e2[i];
        A.a(6, i) = A.a(7, i + 3) = A.a(8, i + 6) = e3[i];
        B[i] = o1[i];
        B[i + 3] = o2[i];
        B[i + 6] = o3[i];
    }
    corecvs::Vector Rv;
    corecvs::Matrix::LinSolve(A, B, Rv);
    int id = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            RC.a(i, j) = Rv[id++];
        }
    }

    corecvs::Vector3dd v;
    corecvs::Matrix33 Vt;
    corecvs::Matrix::svd(&RC, &v, &Vt);

    corecvs::Matrix33 R = RC * Vt.transposed();
    return corecvs::Quaternion::FromMatrix(R);

}

