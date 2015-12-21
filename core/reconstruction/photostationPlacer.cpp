#include "photostationPlacer.h"

#include <unordered_map>
#include <random>

#include "featureMatchingPipeline.h"
#include "essentialEstimator.h"
#include "relativeNonCentralRansacSolver.h"
#include "absoluteNonCentralRansacSolver.h"
#include "bufferReaderProvider.h"
#include "multicameraTriangulator.h"
#include "abstractPainter.h"

#include <tbb/task_group.h>


int corecvs::PhotostationPlacer::getReprojectionCnt()
{
    int total = 0;
    for (auto& o: tracks)
        total += o.projections.size();
    return total * 2;
}

int corecvs::PhotostationPlacer::getOrientationInputNum()
{
    return (preplaced ) * 4 + tracks.size() * 3 + (tuneFocal ? calibratedPhotostations[0].cameras.size() : 0);
}

std::vector<std::vector<int>> corecvs::PhotostationPlacer::getDependencyList()
{
    revDependency.clear();
    int cnt = getReprojectionCnt();
    revDependency.resize(cnt);
    int id = 0;
    for (int i = 0; i < tracks.size(); ++i)
    {
        auto& t = tracks[i];
        for (int j = 0; j < t.projections.size(); ++j)
        {
            revDependency[id++] = std::make_pair(i, j);
            revDependency[id++] = std::make_pair(i, j);
        }
    }
    CORE_ASSERT_TRUE_S(id == cnt);
    std::vector<std::vector<int>> sparsity(getOrientationInputNum());
    int argin = 0;
    int total = 0;
    if (tuneFocal)
    {
        int N = calibratedPhotostations[0].cameras.size();
        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < cnt; ++j)
            {
                auto p = revDependency[j];
                if (tracks[p.first].projections[p.second].cameraId == i)
                    sparsity[argin].push_back(j);
            }
            argin++;
        }
    }
    for (int i = 0; i < preplaced; ++i)
    {
        for (int k = 0; k < 4; ++k)
        {
            for (int j = 0; j < cnt; ++j)
            {
                auto p = revDependency[j];
                if (tracks[p.first].projections[p.second].photostationId == i)
                    sparsity[argin].push_back(j);
            }
            argin++;
        }
    }
    int M = tracks.size();
    for (int i = 0; i < M; ++i)
    {
        for (int k = 0; k < 3; ++k)
        {
            for (int j = 0; j < cnt; ++j)
            {
                auto p = revDependency[j];
                if (p.first == i)
                    sparsity[argin].push_back(j);
            }
            argin++;
        }
    }
    int cnts= 0;
    int K = getOrientationInputNum();
    M = getReprojectionCnt();
    for (int i = 0; i < K; ++i)
        cnts += sparsity[i].size();
    std::cout << "Sparsity: " << (double)cnts / (K * M) << std::endl;
    return sparsity;
}
    
void corecvs::PhotostationPlacer::readOrientationParams(const double in[])
{
    int argin = 0;
    if (tuneFocal)
    {
        for (int i = 0; i < calibratedPhotostations[0].cameras.size(); ++i)
        {
            for (int j = 0; j < calibratedPhotostations.size(); ++j)
                calibratedPhotostations[j].cameras[i].intrinsics.focal = corecvs::Vector2dd(in[argin], in[argin]);
            argin++;
        }
    }
    for (int i = 0; i < preplaced; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            calibratedPhotostations[i].location.rotor[j] = in[argin++];
        }
        calibratedPhotostations[i].location.rotor.normalise();
    }
    for (int j = 0; j < tracks.size(); ++j)
    {
        for (int k = 0; k < 3; ++k)
           tracks[j].worldPoint[k] = in[argin++];
    }
}

void corecvs::PhotostationPlacer::writeOrientationParams(double in[])
{
    int argin = 0;
    if (tuneFocal)
    {
        for (int i = 0;i < calibratedPhotostations[0].cameras.size(); ++i)
            in[argin++] = calibratedPhotostations[0].cameras[i].intrinsics.focal[0];
    }
    for (int i = 0; i < preplaced; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            in[argin++] = calibratedPhotostations[i].location.rotor[j];
        }
    }
    for (int j = 0; j < tracks.size(); ++j)
    {
        for (int k = 0; k < 3; ++k)
           in[argin++] = tracks[j].worldPoint[k];
    }
}

void corecvs::PhotostationPlacer::fitLMedians(bool tuneFocal )
{
    this->tuneFocal = tuneFocal;
    double totalsqr = 0.0;
    double totalcnt = 0.0;
    for (auto& t: tracks)
        for (auto &p : t.projections)
        {
            auto err = p.projection - calibratedPhotostations[p.photostationId].project(t.worldPoint, p.cameraId);
            totalsqr += (!err) * (!err);
            totalcnt++;
        }

    std::cout << "RMSE: " << std::sqrt(totalsqr / totalcnt) << std::endl;
    corecvs::LevenbergMarquardtSparse lm;
    OrientationFunctor orient(this);
    OrientationNormalizationFunctor orientNorm(this);
    lm.f = &orient;
    lm.normalisation = &orientNorm;
    lm.maxIterations = 50;
    lm.trace = false;
    std::vector<double> input(getOrientationInputNum());
    std::vector<double> out(getReprojectionCnt());
    writeOrientationParams(&input[0]);
    auto res = lm.fit(input, out);
    readOrientationParams(&res[0]);

    totalsqr = 0.0;
    totalcnt = 0.0;
    for (auto& t: tracks)
        for (auto &p : t.projections)
        {
            auto err = p.projection - calibratedPhotostations[p.photostationId].project(t.worldPoint, p.cameraId);
            totalsqr += (!err) * (!err);
            totalcnt++;
        }
    std::cout << "RMSE: " << std::sqrt(totalsqr / totalcnt) << std::endl;
    placed = preplaced;
}

void corecvs::PhotostationPlacer::computeMedianErrors(double out[], const std::vector<int> &idxs)
{
#if 0
    std::vector<corecvs::Vector2dd> errors;
    int total = 0;
    for (auto& o: tracks)
    {
//        corecvs::MulticameraTriangulator mct;
//        for (auto& p: o.projections)
//        {
//            mct.addCamera(calibratedPhotostations[p.photostationId].getMMatrix(p.cameraId), p.projection);
//            total += 2;
//        }
//        o.worldPoint = mct.triangulate();
        for (auto& p: o.projections)
            errors.push_back(p.projection - calibratedPhotostations[p.photostationId].project(o.worldPoint, p.cameraId));
    }
    //std::sort(errors.begin(), errors.end(), [](const corecvs::Vector2dd &a, const corecvs::Vector2dd &b) { return !a < !b; });

    int argout = 0;
    for (int i = 0; i < errors.size(); ++i)
    {
        out[argout++] = errors[i][0];
        out[argout++] = errors[i][1];
    }
    while(argout < total / 2)
        out[argout++] = 0.0;
#else
    int argout = 0;
    for (int i = 0; i < idxs.size(); i += 2)
    {
        CORE_ASSERT_TRUE_S(idxs[i] + 1 == idxs[i + 1]);
        auto& o = tracks[revDependency[idxs[i]].first];
        auto& p = o.projections[revDependency[idxs[i]].second];
        auto error = p.projection - calibratedPhotostations[p.photostationId].project(o.worldPoint, p.cameraId);
        out[argout++] = error[0];
        out[argout++] = error[1];
    }
    CORE_ASSERT_TRUE_S(argout == idxs.size());
#endif
}

std::vector<std::tuple<int, corecvs::Vector2dd, int, corecvs::Vector3dd>> corecvs::PhotostationPlacer::getPossibleTracks(int ps)
{
    std::vector<std::tuple<int, corecvs::Vector2dd, int, corecvs::Vector3dd>> res;
    CORE_ASSERT_TRUE_S(ps >= placed);
    for (int psA = 0; psA < placed; ++psA)
    {
        int id2 = ps - psA;
        auto& mm = matches[psA][id2];
        for (auto& m: mm)
        {
            int camA = std::get<0>(m);
            int  ptA = std::get<1>(m);
            int camB = std::get<2>(m);
            int  ptB = std::get<3>(m);
            if (!trackMap.count(std::make_tuple(psA, camA, ptA)))
                continue;
            int track = trackMap[std::make_tuple(psA, camA, ptA)];
            res.emplace_back(camB, keyPoints[ps][camB][ptB], track, tracks[track].worldPoint);

        }
    }
    return res;
}
#if 0
void corecvs::PhotostationPlacer::pruneTrachs()
{
    double targetRMSE, currentRMSE;

}
#endif

void corecvs::PhotostationPlacer::appendTracks(const std::vector<int> &inlierIds, int ps)
{
    int inlierIdx = 0;
    int totalIdx = 0;
    int appended = 0;
    for (int psA = 0; psA < placed; ++psA)
    {
        int id2 = ps - psA;
        auto& mm = matches[psA][id2];
        for (auto& m: mm)
        {
            int camA = std::get<0>(m);
            int  ptA = std::get<1>(m);
            int camB = std::get<2>(m);
            int  ptB = std::get<3>(m);
            if (!trackMap.count(std::make_tuple(psA, camA, ptA)))
                continue;
            int track = trackMap[std::make_tuple(psA, camA, ptA)];
            if (inlierIdx >= inlierIds.size() || totalIdx++ != inlierIds[inlierIdx])
            {
                continue;
            }
            inlierIdx++;
            if (trackMap.count(std::make_tuple(ps, camB, ptB)))
                continue;
            auto wp = tracks[track].worldPoint;
            auto proj = keyPoints[ps][camB][ptB] - calibratedPhotostations[ps].project(wp, camB);
            if ((!proj) < trackInlierThreshold && (!(wp - calibratedPhotostations[ps].getRawCamera(camB).extrinsics.position) < distanceLimit))
            {
                PointProjection prj;
                prj.photostationId = ps;
                prj.cameraId = camB;
                prj.projection = keyPoints[ps][camB][ptB];
                prj.featureId = ptB;
                appended++;
                tracks[track].projections.push_back(prj);
                trackMap[std::make_tuple(ps, camB, ptB)] = track;
            }
        }
    }
    std::cout << appended << " tracks appended" << std::endl;
}

void corecvs::PhotostationPlacer::appendPs()
{
    int psApp = placed;
    std::cout << "Placing #" << placed << std::endl;
    auto hypos = getPossibleTracks(psApp);
    std::cout << "Total " << hypos.size() << " possible tracks" << std::endl;

//    AbsoluteNonCentralRansacSolver solver(calibratedPhotostations[psApp], hypos);
    corecvs::AbsoluteNonCentralRansacSolverParams params;
    params.forcePosition = true;
    params.forcedPosition = gpsData[psApp];
    AbsoluteNonCentralRansacSolver solver(calibratedPhotostations[psApp], hypos, params);
    solver.run();
//    solver.runInliersPNP();
    solver.runInliersRE();
    auto res = solver.getInliers();
    auto hypo = solver.getBestHypothesis();
    std::cout << "PS append: " << res.size() << std::endl;
    std::cout << hypo.shift << " " << hypo.rotor << " | " << gpsData[psApp] << std::endl;
    calibratedPhotostations[psApp].location = hypo;
    calibratedPhotostations[psApp].location.shift = gpsData[psApp];
    appendTracks(res, psApp);
    preplaced++;
}

void corecvs::PhotostationPlacer::buildTracks(int psA, int psB, int psC)
{
    bool swapAB = psB < psA;
    int id1AB = std::min(psA, psB);
    int id2AB = std::max(psA, psB) - id1AB;
    bool swapBC = psC < psB;
    int id1BC = std::min(psB, psC);
    int id2BC = std::max(psB, psC) - id1BC;
    bool swapAC = psC < psA;
    int id1AC = std::min(psA, psC);
    int id2AC = std::max(psA, psC) - id1AC;

    auto& matchesAB = matches[id1AB][id2AB];
    auto& matchesBC = matches[id1BC][id2BC];
    auto& matchesAC = matches[id1AC][id2AC];

    std::unordered_map<std::tuple<int, int, int>, int> freeAC, freeAB, freeBC;
    for (auto& mAB: matchesAB)
    {
        int camA = swapAB ? std::get<2>(mAB) : std::get<0>(mAB);
        int  ptA = swapAB ? std::get<3>(mAB) : std::get<1>(mAB);
        int camB =!swapAB ? std::get<2>(mAB) : std::get<0>(mAB);
        int  ptB =!swapAB ? std::get<3>(mAB) : std::get<1>(mAB);
        if (!trackMap.count(std::make_tuple(psA, camA, ptA)) && !trackMap.count(std::make_tuple(psB, camB, ptB)))
        {
            freeAB[std::make_tuple(camA, ptA, camB)] = ptB;
        }
    }
    for (auto& mBC: matchesBC)
    {
        int camB = swapBC ? std::get<2>(mBC) : std::get<0>(mBC);
        int  ptB = swapBC ? std::get<3>(mBC) : std::get<1>(mBC);
        int camC =!swapBC ? std::get<2>(mBC) : std::get<0>(mBC);
        int  ptC =!swapBC ? std::get<3>(mBC) : std::get<1>(mBC);
        if (!trackMap.count(std::make_tuple(psB, camB, ptB)) && !trackMap.count(std::make_tuple(psC, camC, ptC)))
        {
            freeBC[std::make_tuple(camB, ptB, camC)] = ptC;
        }
    }
    for (auto& mAC: matchesAC)
    {
        int camA = swapAC ? std::get<2>(mAC) : std::get<0>(mAC);
        int  ptA = swapAC ? std::get<3>(mAC) : std::get<1>(mAC);
        int camC =!swapAC ? std::get<2>(mAC) : std::get<0>(mAC);
        int  ptC =!swapAC ? std::get<3>(mAC) : std::get<1>(mAC);
        if (!trackMap.count(std::make_tuple(psA, camA, ptA)) && !trackMap.count(std::make_tuple(psC, camC, ptC)))
        {
            freeAC[std::make_tuple(camA, ptA, camA)] = ptC;
        }
    }
    std::vector<std::tuple<int, int, int, int, int, int>> trackCandidates;
    for (auto& mAC: freeAC)
    {
        int camA = std::get<0>(mAC.first),
             ptA = std::get<1>(mAC.first),
            camC = std::get<2>(mAC.first),
             ptC = mAC.second;
        for (int camB = 0; camB < calibratedPhotostations[psB].cameras.size(); ++camB)
        {
            if (freeAB.count(std::make_tuple(camA, ptA, camB)))
            {
                int ptB = freeAB[std::make_tuple(camA, ptA, camB)];
                if (freeBC.count(std::make_tuple(camB, ptB, camC)))
                {
                    int ptC2 = freeBC[std::make_tuple(camB, ptB, camC)];
                    if (ptC2 == ptC)
                        trackCandidates.emplace_back(camA, ptA, camB, ptB, camC, ptC);
                }
            }
        }
    }

    for (auto& c: trackCandidates)
    {
        int camA = std::get<0>(c),
             ptA = std::get<1>(c),
            camB = std::get<2>(c),
             ptB = std::get<3>(c),
            camC = std::get<4>(c),
             ptC = std::get<5>(c);
        auto kpA = keyPoints[psA][camA][ptA];
        auto kpB = keyPoints[psB][camB][ptB];
        auto kpC = keyPoints[psC][camC][ptC];
        if (trackMap.count(std::make_tuple(psA, camA, ptA))||
            trackMap.count(std::make_tuple(psB, camB, ptB))||
            trackMap.count(std::make_tuple(psC, camC, ptC)))
            continue;

        double FAB = scoreFundamental(psA, camA, kpA, psB, camB, kpB);
        double FBC = scoreFundamental(psB, camB, kpB, psC, camC, kpC);
        double FAC = scoreFundamental(psA, camA, kpA, psC, camB, kpC);
        if (std::max(FAB, std::max(FBC, FAC)) < trackInlierThreshold)
        {
            corecvs::MulticameraTriangulator mct;
            mct.addCamera(calibratedPhotostations[psA].getMMatrix(camA), kpA);
            mct.addCamera(calibratedPhotostations[psB].getMMatrix(camB), kpB);
            mct.addCamera(calibratedPhotostations[psC].getMMatrix(camC), kpC);
            auto res = mct.triangulateLM(mct.triangulate());
#if 0
            if (!(res - calibratedPhotostations[psA].location.shift) > 500 ||
            	!(res - calibratedPhotostations[psB].location.shift) > 500 ||
            	!(res - calibratedPhotostations[psC].location.shift) > 500)
            	continue;
#endif
            if (!calibratedPhotostations[psA].isVisible(res, camA)
             || !calibratedPhotostations[psB].isVisible(res, camB)
             || !calibratedPhotostations[psC].isVisible(res, camC))
                 continue;
            PointObservation__ po;
            PointProjection pa = { kpA, psA, camA, ptA },
                            pb = { kpB, psB, camB, ptB },
                            pc = { kpC, psC, camC, ptC };

            po.projections.push_back(pa);
            po.projections.push_back(pb);
            po.projections.push_back(pc);
            po.worldPoint = res;
            int id = tracks.size();
            if (!(kpA - calibratedPhotostations[psA].project(res, camA)) > trackInlierThreshold ||
                !(kpB - calibratedPhotostations[psB].project(res, camB)) > trackInlierThreshold ||
                !(kpC - calibratedPhotostations[psC].project(res, camC)) > trackInlierThreshold)
                continue;
            if (
                    !(res - calibratedPhotostations[psA].getRawCamera(camA).extrinsics.position) > distanceLimit ||
                    !(res - calibratedPhotostations[psB].getRawCamera(camB).extrinsics.position) > distanceLimit ||
                    !(res - calibratedPhotostations[psC].getRawCamera(camC).extrinsics.position) > distanceLimit)
                continue;
            tracks.push_back(po);
            trackMap[std::make_tuple(psA, camA, ptA)] = id;
            trackMap[std::make_tuple(psB, camB, ptB)] = id;
            trackMap[std::make_tuple(psC, camC, ptC)] = id;
        }
    }
}

double corecvs::PhotostationPlacer::scoreFundamental(int psA, int camA, corecvs::Vector2dd ptA,
                                 int psB, int camB, corecvs::Vector2dd ptB)
{
    auto FAB = calibratedPhotostations[psA].getRawCamera(camA).fundamentalTo(
               calibratedPhotostations[psB].getRawCamera(camB));
    corecvs::Line2d left = FAB.mulBy2dRight(ptB);
    corecvs::Line2d right= FAB.mulBy2dLeft (ptA);
    return std::max(left.distanceTo(ptA), right.distanceTo(ptB));

}



void corecvs::PhotostationPlacer::backprojectAll()
{
    int idx = 0;
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
                {
                    backprojected[idx].emplace_back(calibratedPhotostations[psA].getRawCamera(camA).extrinsics.position, res);
//                    backprojected.emplace_back(calibratedPhotostations[psB].getRawCamera(camB).extrinsics.position, res);
                }
            }
                idx++;
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
                E.getScaler(pE1, pE2, sL, sR, foo);
                if (sL < 0 || sR < 0 || std::max(left, right) > inlierThreshold)
                    outliers.push_back(i);

            }

            remove(psA, camA, psB, camB, outliers);
        }
    }
}

#if 0
void corecvs::PhotostationPlacer::printMatchStats()
{
    for (int psA = 0; psA < calibratedPhotostations.size(); ++psA)
    {
        for (int psB = psA + 1; psB < calibratedPhotostations.size(); ++psB)
        {
        }
    }
}
#endif
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
        ps.location.shift *= 1e3;
        for (int j = 0; j < ps.cameras.size(); ++j)
            ps.cameras[j].extrinsics.position *= 1e3;
        ps.name = ss.str();

        CalibrationHelpers().drawPly(meshres, ps, 50.0);
    }
    meshres.dumpPLY("triples_before_reorient.ply");



    auto q = detectOrientationFirst();
    for (int psA = 0; psA < 3; ++psA)
        for (int psB = 0; psB < 3; ++psB)
    std::cout << calibratedPhotostations[psA].getRawCamera(0).rayFromPixel(corecvs::Vector2dd(0, 0)).a.angleTo(calibratedPhotostations[psB].location.shift - calibratedPhotostations[psA].location.shift) << std::endl;
    calibratedPhotostations[0].location.rotor = q.conjugated();
    calibratedPhotostations[0].location.shift = gpsData[0];
    calibratedPhotostations[1].location.rotor = q.conjugated() ^ calibratedPhotostations[1].location.rotor;
    calibratedPhotostations[1].location.shift = gpsData[1];
    calibratedPhotostations[2].location.rotor = q.conjugated() ^ calibratedPhotostations[2].location.rotor;
    calibratedPhotostations[2].location.shift = gpsData[2];
    for (int psA = 0; psA < 3; ++psA)
        for (int psB = 0; psB < 3; ++psB)
    std::cout << calibratedPhotostations[psA].getRawCamera(0).rayFromPixel(corecvs::Vector2dd(0, 0)).a.angleTo(calibratedPhotostations[psB].location.shift - calibratedPhotostations[psA].location.shift) << std::endl;

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
    auto Rv = corecvs::Matrix::LinSolve(A, B);
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
    best = solver.getBestHypothesis();
    std::cout << psA << "::" << psB << " " << best.shift << " " << best.rotor << std::endl;
    calibratedPhotostations[psB].location = best;
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

                        decompositions[i].getScaler(L1, R1, scaleL, scaleR, foo);
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
    for (int psA = 0; psA < 3; ++psA)
    {
        for (int psB = psA; psB < 3; ++psB)
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

std::vector<std::vector<PointObservation__>> corecvs::PhotostationPlacer::verify(const std::vector<PointObservation__> &pois)
{
    BufferReader* reader = BufferReaderProvider::getInstance().getBufferReader(images[0][0]);
						corecvs::RGBColor
							poi(0x00ff0000),
							ppo(0x0000ff00),
							ppt(0x000000ff);
						std::cout << "Marking POI projection on cameras as " << poi << std::endl << "Marking POI observed projection as " << ppo << std::endl << "Marking POI triangulated projection as " << ppt << std::endl;
						std::cout << "R=" << (int)poi.r() << " G=" << (int)poi.g() << " B= " << (int)poi.b() << std::endl;
						std::cout << "R=" << (int)ppo.r() << " G=" << (int)ppo.g() << " B= " << (int)ppo.b() << std::endl;
						std::cout << "R=" << (int)ppt.r() << " G=" << (int)ppt.g() << " B= " << (int)ppt.b() << std::endl;
	for (int i = 0; i < calibratedPhotostations.size(); ++i)
	{
		for (int j = 0; j < images[i].size(); ++j)
		{
			corecvs::RGB24Buffer buffer = reader->readRgb(images[i][j]);
			for (auto& o: pois)
			{
				corecvs::MulticameraTriangulator mct;
				for (auto& p: o.projections)
				{
					mct.addCamera(calibratedPhotostations[p.photostationId].getMMatrix(p.cameraId), p.projection);
				}
				auto pptt = mct.triangulateLM(mct.triangulate());
				for (auto& p: o.projections)
				{
					if (p.photostationId == i && p.cameraId == j)
					{
						auto proj = p.projection;
						auto wpt  = o.worldPoint;
						auto proj2= calibratedPhotostations[i].project(wpt, p.cameraId);
						auto projt= calibratedPhotostations[i].project(pptt,p.cameraId);
						for (int xx = -15; xx <= 15; ++xx)
							for (int yy = -15; yy <= 15; ++yy)
							{
								if (xx != 0 && yy != 0)
									continue;
								int x, y;
								x = proj[0] - xx; y = proj[1] - yy;
								if (x >= 0 && y >= 0 && x < buffer.w && y < buffer.h)
									buffer.element(y, x) = poi;
								x = proj2[0] - xx; y = proj2[1] - yy;
								if (x >= 0 && y >= 0 && x < buffer.w && y < buffer.h)
									buffer.element(y, x) = ppo;
								x = projt[0] - xx; y = projt[1] - yy;
								if (x >= 0 && y >= 0 && x < buffer.w && y < buffer.h)
									buffer.element(y, x) = ppt;

							}
                        corecvs::AbstractPainter<corecvs::RGB24Buffer> painter(&buffer);
                        painter.drawFormat(proj[0] + 10, proj[1] + 10, poi, 6, "%s (marked poi)", o.label.c_str());
                        painter.drawFormat(proj2[0] + 10, proj2[1] + 10, ppo, 6, "%s (wp-projection)", o.label.c_str());
                        painter.drawFormat(projt[0] + 10, projt[1] + 10, ppt, 6, "%s (tri-projection)", o.label.c_str());
					}
				}
			}
			std::string filename = images[i][j];
			filename.resize(filename.size() - 3);
			filename = filename + "_pois.jpg";
			reader->writeRgb(buffer, filename); 
		}
	}
	std::vector<std::vector<PointObservation__>> res(2);
	for (auto& obs: pois)
	{
		auto wp = obs.worldPoint;
		auto obs1 = obs;
		auto obs2 = obs;
		corecvs::MulticameraTriangulator mct2;
		for (auto& p: obs1.projections)
		{
			p.projection = calibratedPhotostations[p.photostationId].project(wp, p.cameraId);
			mct2.addCamera(calibratedPhotostations[p.photostationId].getMMatrix(p.cameraId), p.projection);
		}
		obs1.worldPoint = mct2.triangulateLM(mct2.triangulate());
		corecvs::MulticameraTriangulator mct;
		for (auto& p: obs2.projections)
		{
			mct.addCamera(calibratedPhotostations[p.photostationId].getMMatrix(p.cameraId), p.projection);
		}
		obs2.worldPoint = mct.triangulateLM(mct.triangulate());
		for (auto& p: obs2.projections)
		{
			p.projection = calibratedPhotostations[p.photostationId].project(obs2.worldPoint, p.cameraId);
		}
		res[0].push_back(obs1);
		res[1].push_back(obs2);
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
    keyPointColors.clear();
    keyPoints.resize(calibratedPhotostations.size());
    keyPointColors.resize(calibratedPhotostations.size());

    for (int i = 0; i < keyPoints.size(); ++i)
    {
        keyPoints[i].resize(calibratedPhotostations[i].cameras.size());
        keyPointColors[i].resize(calibratedPhotostations[i].cameras.size());
    }
    int N = filenames.size();
    for (int i = 0; i < N; ++i)
    {
        auto& kps = pipeline.images[i].keyPoints.keyPoints;
        auto id = img_map[i];
        for (auto& kp: kps)
        {
            keyPoints[id.first][id.second].emplace_back(kp.x, kp.y);
            keyPointColors[id.first][id.second].push_back(kp.color);
//                std::cout << "R = " << (int)kp.color.r() << " G = " << (int)kp.color.g() << " B = " << (int)kp.color.b() << std::endl;
        }
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
