#include "photostationPlacer.h"

#include <unordered_map>
#include <random>

#include "featureMatchingPipeline.h"
#include "essentialEstimator.h"
#include "relativeNonCentralRansacSolver.h"
#include "multicameraTriangulator.h"

#include <tbb/task_group.h>

void corecvs::PhotostationPlacer::backprojectAll()
{
    for (int psA = placed; psA < preplaced; ++psA)
    {
        for (int psB = psA + 1; psB < preplaced; ++psB)
        {
            auto m = getPhotostationMatches(psA, psB);
            for (auto &mm: m)
            {
                int camA = std::get<0>(mm);
                int camB = std::get<2>(mm);
                auto ptA = std::get<1>(mm);
                auto ptB = std::get<3>(mm);
                corecvs::MulticameraTriangulator mct;
                mct.addCamera(calibratedPhotostations[psA].getMMatrix(camA), ptA);
                mct.addCamera(calibratedPhotostations[psB].getMMatrix(camB), ptB);
                auto res = mct.triangulate();
                if (calibratedPhotostations[psA].isVisible(res, camA)
                &&  calibratedPhotostations[psB].isVisible(res, camB))
                    backprojected.push_back(res);
            }
        }
    }
}

void corecvs::PhotostationPlacer::selectEpipolarInliers(int psA, int psB)
{
    for (int camA = 0; camA < calibratedPhotostations[psA].cameras.size(); ++camA)
    {
        for (int camB = 0; camB < calibratedPhotostations[psB].cameras.size(); ++camB)
        {
            corecvs::Matrix33 F = calibratedPhotostations[psA].getRawCamera(camA).fundamentalTo(calibratedPhotostations[psB].getRawCamera(camB));
            corecvs::EssentialDecomposition E = calibratedPhotostations[psA].getRawCamera(camA).essentialDecomposition(calibratedPhotostations[psB].getRawCamera(camB));
            auto K1 = calibratedPhotostations[psA].getRawCamera(camA).intrinsics.getKMatrix33().inv();
            auto K2 = calibratedPhotostations[psB].getRawCamera(camB).intrinsics.getKMatrix33().inv();
            auto mc = getCameraMatches(psA, camA, psB, camB);
            std::vector<int> outliers;
            
            for (int i = 0; i < mc.size(); ++i)
            {
                auto m = mc[i];
                auto p1 = std::get<0>(m);
                auto p2 = std::get<1>(m);
                auto pE1 = K1 * p1;
                auto pE2 = K2 * p2;


                corecvs::Line2d lineLeft(F.mulBy2dRight(p2));
                corecvs::Line2d lineRight(F.mulBy2dLeft(p1));
                double left = lineLeft.distanceTo(p1);
                double right= lineRight.distanceTo(p2);
                double sL, sR, foo;
                E.getScaler(pE2, pE1, sL, sR, foo);
                if (sL < 0 || sR < 0 || std::max(left, right) > inlierThreshold)
                    outliers.push_back(i);

            }

            remove(psA, camA, psB, camB, outliers);
        }
    }
}

void corecvs::PhotostationPlacer::selectEpipolarInliers()
{
    matchesCopy = matches;
    for (int psA = placed; psA < preplaced; ++psA)
    {
        for (int psB = psA + 1; psB < preplaced; ++psB)
        {
            selectEpipolarInliers(psA, psB);
        }
    }
}

#include "calibrationHelpers.h"
#include "calibrationLocation.h"
#include "mesh3d.h"
#include <sstream>

std::atomic<int> corecvs::PhotostationPlacer::ParallelEssentialFilter::cntr;
void corecvs::PhotostationPlacer::estimateFirstPair()
{
    tbb::task_group g;
    g.run([=]() { estimatePair(0, 1); });
    g.run([=]() { estimatePair(0, 2); });
    g.wait();


    corecvs::Mesh3D meshres;
    meshres.switchColor(true);
    corecvs::Vector3dd meanpos(0, 0, 0);
    for (int i = 0; i < 3; ++i)
    {
        meanpos += calibratedPhotostations[i].location.shift * (1.0 / 3);
    }
    for (int i = 0; i < 3; ++i)
    {
        std::stringstream ss;
        ss << "SP" << ((char)('A' + i));
        corecvs::Photostation ps = calibratedPhotostations[i];
        ps.location.shift -= meanpos;
        ps.name = ss.str();

        CalibrationHelpers().drawPly(meshres, ps, 1.0);
    }
    meshres.dumpPLY("triples_before_reorient.ply");



    auto q = detectOrientationFirst();
    calibratedPhotostations[0].location.rotor = q.conjugated();
    calibratedPhotostations[0].location.shift = gpsData[0];
    calibratedPhotostations[1].location.rotor = q.conjugated() ^ calibratedPhotostations[1].location.rotor;
    calibratedPhotostations[1].location.shift = gpsData[1];
    calibratedPhotostations[2].location.rotor = q.conjugated() ^ calibratedPhotostations[2].location.rotor;
    calibratedPhotostations[2].location.shift = gpsData[2];
    preplaced = 3;
    matches = matchesCopy;
}

corecvs::Quaternion corecvs::PhotostationPlacer::detectOrientationFirst()
{
    std::cout << "ABC: " << (gpsData[1] - gpsData[0]).angleTo(gpsData[2] - gpsData[0]) << " | " << calibratedPhotostations[1].location.shift.angleTo(calibratedPhotostations[2].location.shift) << std::endl;

    corecvs::Vector3dd e1 = gpsData[1] - gpsData[0];
    corecvs::Vector3dd e2 = gpsData[2] - gpsData[0];

    corecvs::Vector3dd o1 = calibratedPhotostations[1].location.shift;
    corecvs::Vector3dd o2 = calibratedPhotostations[2].location.shift;

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
    auto Rv = corecvs::Matrix::linSolve(A, B);
    int id = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            RC.a(i, j) = Rv[id++];
        }
    }

    corecvs::Vector3dd v;
    corecvs::Matrix33 TT=RC, Vt, D(0, 0, 0, 0, 0, 0, 0, 0, 0);
    corecvs::Matrix::svd(&RC, &v, &Vt);
    D.a(0, 0) = v[0];
    D.a(1, 1) = v[1];
    D.a(2, 2) = v[2];
    
    corecvs::Matrix33 R = RC * Vt.transposed();
    return corecvs::Quaternion::FromMatrix(R);
}

void corecvs::PhotostationPlacer::estimatePair(int psA, int psB)
{
    auto matches = getPhotostationMatches(psA, psB);
    RelativeNonCentralRansacSolver::MatchContainer rm, mm;
    for (auto&t : matches)
    {
        if (std::get<4>(t) < b2bRansacP6RPThreshold)
            rm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
        mm.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t));
    }
    
    calibratedPhotostations[psA].location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    calibratedPhotostations[psA].location.shift = corecvs::Vector3dd(0, 0, 0);

    RelativeNonCentralRansacSolver solver(
            calibratedPhotostations[psA],
            calibratedPhotostations[psB], rm, mm);
    solver.run();
    auto best = solver.getBestHypothesis();
    std::cout << psA << "::" << psB << " " << best.shift << " " << best.rotor << std::endl;
//    solver.fit(!(gpsData[psA] - gpsData[psB]));
//    best = solver.getBestHypothesis();
//    std::cout << psA << "::" << psB << " " << best.shift << " " << best.rotor << std::endl;
    calibratedPhotostations[psB].location = best;
    calibratedPhotostations[psB].location.rotor = calibratedPhotostations[psB].location.rotor.conjugated();
    pairInliers.emplace_back(psA, psB, solver.getBestInliers());
}

void corecvs::PhotostationPlacer::filterEssentialRansac(int psA, int camA, int psB, int camB)
{
    auto matches = getCameraMatches(psA, camA, psB, camB);
    std::vector<int> ransacableIdx;
    for (auto&t: matches)
        if (std::get<2>(t) < b2bRansacP5RPThreshold)
            ransacableIdx.push_back(&t - &matches[0]);
    
    int bestInlierCnt = 0;
    std::vector<int> bestInliers;

    std::mt19937 rng;
    int N = ransacableIdx.size();
    std::cout << psA << ":" << camA << "<>" << psB << ":" << camB << ": " << ransacableIdx.size() << std::endl;
    if (N == 0)
        return;
    auto K1 = calibratedPhotostations[psA].getRawCamera(camA).intrinsics.getKMatrix33().inv();
    auto K2 = calibratedPhotostations[psB].getRawCamera(camB).intrinsics.getKMatrix33().inv();

    for (int it = 0; it < maxEssentialRansacIterations; ++it)
    {
        const int NSEL = 5;
        int idxs[NSEL];
        int rdy = 0;
        for (; rdy < NSEL;)
        {
            int idx = ransacableIdx[rng() % N];
            bool ok = true;
            for (int i = 0; i < rdy && ok; ++i)
                if (idxs[i] == idx)
                    ok = false;
            if (!ok) continue;
            idxs[rdy++] = idx;
        }
       
        std::vector<corecvs::Correspondence> cv;
        cv.reserve(NSEL);
        for (auto& id: idxs)
        {
            corecvs::Correspondence corr;
            corr.start = K1 * std::get<0>(matches[id]);
            corr.end   = K2 * std::get<1>(matches[id]);
            cv.push_back(corr);
        }
        std::vector<Correspondence*> cl;
        for (auto& cc: cv)
            cl.push_back(&cc);

        auto Fv = corecvs::EssentialEstimator().getEssential5point(cl);

        for (auto& F: Fv)
        {
            EssentialDecomposition decompositions[4];
            F.decompose(decompositions);
            int inlierCnt[4] = {0};
            corecvs::Matrix33 FM = K1.transposed() * F * K2;
            std::vector<int> inliers[4];
            for (auto &m: matches)
            {
                corecvs::Vector2dd L = std::get<0>(m);
                corecvs::Vector2dd R = std::get<1>(m);
                corecvs::Line2d lineL = FM.mulBy2dRight(R);
                corecvs::Line2d lineR = FM.mulBy2dLeft(L);
                double diff = std::max(lineL.distanceTo(L), lineR.distanceTo(R));

                if (diff < inlierP5RPThreshold)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        double scaleL, scaleR, foo;
                        auto R1 = K2 * R;
                        auto L1 = K1 * L;

                        decompositions[i].getScaler(R1, L1, scaleL, scaleR, foo);
                        if (scaleL > 0.0 && scaleR > 0.0)
                        {
                            ++inlierCnt[i];
                            inliers[i].push_back(&m - &matches[0]);
                        }
                    }
                }
            }
            for (int i = 0; i < 4; ++i)
            {
                if (inlierCnt[i] > bestInlierCnt)
                {
                    bestInlierCnt = inlierCnt[i];
                    bestInliers = std::move(inliers[i]);
                }
            }
        }
    }
    std::cout << "ESSF: " << psA << "|" << camA << " <> " << psB << "|" << camB << " " << bestInliers.size() << " (" << matches.size() << ") " << ((double)bestInliers.size())/(double)matches.size() << std::endl;
    std::vector<int> delIdx;
    std::sort(bestInliers.begin(), bestInliers.end());
    int inlierId = 0;
    for (int i = 0; i < matches.size(); ++i)
    {
        if (inlierId < bestInliers.size() && bestInliers[inlierId] == i)
        {
            inlierId++;
            continue;
        }
        delIdx.push_back(i);
    }
    tbb::mutex::scoped_lock(mutex);
    remove(psA, camA, psB, camB, delIdx);
}

void corecvs::PhotostationPlacer::filterEssentialRansac()
{
    matchesCopy = matches;
    std::vector<std::tuple<int, int, int, int>> work;
    for (int psA = 0; psA < calibratedPhotostations.size(); ++psA)
    {
        for (int psB = psA; psB < calibratedPhotostations.size(); ++psB)
        {
            for (int camA = 0; camA < calibratedPhotostations[psA].cameras.size(); ++camA)
            {
                for (int camB = 0; camB < calibratedPhotostations[psB].cameras.size(); ++camB)
                {
                    work.emplace_back(psA, camA, psB, camB);
//                    filterEssentialRansac(psA, camA, psB, camB);
                }
            }
        }
    }
    corecvs::parallelable_for(0, (int)work.size(), ParallelEssentialFilter(this, work));
}

void corecvs::PhotostationPlacer::remove(int psA, int psB, std::vector<int> idx)
{
    bool swap = psA > psB;
    int id1 = swap ? psB : psA;
    int id2 = swap ? psA : psB;
    int okay = 0;
    std::sort(idx.begin(), idx.end());
    int idxSkip = 0;
    for (int i = 0; i < matches[id1][id2 - id1].size(); ++i)
    {
        if (idxSkip < idx.size() && i == idx[idxSkip])
        {
            idxSkip++;
            continue;
        }
        matches[id1][id2 - id1][okay++] = matches[id1][id2 - id1][i];
    }
    matches[id1][id2 - id1].resize(okay);
}

void corecvs::PhotostationPlacer::remove(int psA, int camA, int psB, int camB, std::vector<int> idx)
{
    auto psps = getPhotostationMatches(psA, psB);
    std::sort(idx.begin(), idx.end());
    int idxSkip = 0;
    std::vector<int> skipGlobal;
    int idxCams = 0;
    for (int i = 0; i < psps.size(); ++i)
    {
        auto t = psps[i];
        int cam1 = std::get<0>(t);
        int cam2 = std::get<2>(t);
        if (cam1 != camA || cam2 != camB)
            continue;
        if (idxSkip < idx.size() && idx[idxSkip] == idxCams)
        {
            idxSkip++;
            skipGlobal.push_back(i);
            idxCams++;
            continue;
        }
        idxCams++;
    }
}


std::vector<std::tuple<int, corecvs::Vector2dd, int, corecvs::Vector2dd, double>> 
corecvs::PhotostationPlacer::getPhotostationMatches(int psA, int psB)
{
    bool swap = psA > psB;
    std::vector<std::tuple<int, corecvs::Vector2dd, int, corecvs::Vector2dd, double>> res;
    int id1 = swap ? psB : psA;
    int id2 = swap ? psA : psB;
    for (int i = 0; i < matches[id1][id2 - id1].size(); ++i)
    {
        auto t = matches[id1][id2 - id1][i];
        int cam1 = std::get<0>(t);
        int cam2 = std::get<2>(t);
        int pt1 = std::get<1>(t);
        int pt2 = std::get<3>(t);
        double dist = std::get<4>(t);
        if (!swap)
            res.emplace_back(cam1, keyPoints[id1][cam1][pt1], cam2, keyPoints[id2][cam2][pt2], dist);
        else
            res.emplace_back(cam2, keyPoints[id2][cam2][pt2], cam1, keyPoints[id1][cam1][pt1], dist);
    }
    return res;
}

std::vector<std::tuple<corecvs::Vector2dd, corecvs::Vector2dd, double>> 
corecvs::PhotostationPlacer::getCameraMatches(int psA, int camA, int psB, int camB)
{
    auto psps = getPhotostationMatches(psA, psB);
    std::vector<std::tuple<corecvs::Vector2dd, corecvs::Vector2dd, double>> res;
    for (auto&m: psps)
    {
        if (std::get<0>(m) == camA && std::get<2>(m) == camB)
            res.emplace_back(std::get<1>(m), std::get<3>(m), std::get<4>(m));
    }
    return res;
}

void corecvs::PhotostationPlacer::detectAll()
{
    std::vector<std::string> filenames;
    std::unordered_map<int, std::pair<int, int>> img_map;
    for (int i = 0; i < images.size(); ++i)
    {
        for (int j = 0; j < images[i].size(); ++j)
        {
            filenames.push_back(images[i][j]);
            img_map[filenames.size() - 1] = std::make_pair(i, j);
        }
    }

    FeatureMatchingPipeline pipeline(filenames);
    pipeline.add(new KeyPointDetectionStage(detector), true);
    pipeline.add(new DescriptorExtractionStage(descriptor), true);
    pipeline.add(new MatchingPlanComputationStage(), true);
    pipeline.add(new MatchAndRefineStage(descriptor, matcher, b2bThreshold), true);
    pipeline.run();

    keyPoints.clear();
    keyPoints.resize(calibratedPhotostations.size());
    for (int i = 0; i < keyPoints.size(); ++i)
        keyPoints[i].resize(calibratedPhotostations[i].cameras.size());
    int N = filenames.size();
    for (int i = 0; i < N; ++i)
    {
        auto& kps = pipeline.images[i].keyPoints.keyPoints;
        auto id = img_map[i];
        for (auto& kp: kps)
            keyPoints[id.first][id.second].emplace_back(kp.x, kp.y);
    }
    
    matches.clear();

    matches.resize(calibratedPhotostations.size());
    for (int i = 0; i < calibratedPhotostations.size(); ++i)
        matches[i].resize(calibratedPhotostations.size() - i);

    auto& ref = pipeline.refinedMatches.matchSets;
    for (auto& ms: ref)
    {
        auto id1 = img_map[ms.imgA];
        auto id2 = img_map[ms.imgB];
        bool swap = id1.first > id2.first;
        int psIdA = swap ? id2.first : id1.first;
        int psIdB = swap ? id1.first : id2.first;
        int camIdA = swap ? id2.second : id1.second;
        int camIdB = swap ? id1.second : id2.second;

        for(auto& m:ms.matches)
        {
            int featureA = swap ? m.featureB : m.featureA;
            int featureB = swap ? m.featureA : m.featureB;
            matches[psIdA][psIdB - psIdA].emplace_back(camIdA, featureA, camIdB, featureB, m.best2ndBest);
        }
    }
}
