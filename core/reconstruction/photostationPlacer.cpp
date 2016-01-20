#include "photostationPlacer.h"

#include <unordered_map>
#include <random>
#include <sstream>

#include "featureMatchingPipeline.h"
#include "essentialEstimator.h"
#include "relativeNonCentralRansacSolver.h"
#include "absoluteNonCentralRansacSolver.h"
#include "bufferReaderProvider.h"
#include "multicameraTriangulator.h"
#include "pnpSolver.h"
#include "abstractPainter.h"
#include "calibrationHelpers.h"
#include "calibrationLocation.h"


#ifdef WITH_TBB
#include <tbb/task_group.h>
#endif
#if 0
#define IFNOT(cond, expr) \
    if (!(optimizationParams & PhotostationPlacerOptimizationType::cond)) \
    { \
        expr; \
    }
#define IF(cond, expr) \
    if (!!(optimizationParams & PhotostationPlacerOptimizationType::cond)) \
    { \
        expr; \
    }
#define GETPARAM(ref) \
    ref = in[argin++];
#define IF_GETPARAM(cond, ref) \
    if (!!(optimizationParams & PhotostationPlacerOptimizationType::cond)) ref = in[argin++];
#define IFNOT_GETPARAM(cond, ref) \
    if ( !(optimizationParams & PhotostationPlacerOptimizationType::cond)) ref = in[argin++];
#define SETPARAM(ref) \
    out[argout++] = ref;
#define IF_SETPARAM(cond, ref) \
    if (!!(optimizationParams & PhotostationPlacerOptimizationType::cond)) out[argout++] = ref;
#define IFNOT_SETPARAM(cond, ref) \
    if ( !(optimizationParams & PhotostationPlacerOptimizationType::cond)) out[argout++] = ref;

std::string toString(PhotostationPlacerOptimizationErrorType type)
{
    switch(type)
    {
        case PhotostationPlacerOptimizationErrorType::REPROJECTION:
            return "REPROJECTION";
        case PhotostationPlacerOptimizationErrorType::ANGULAR:
            return "ANGULAR";
        case PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT:
            return "CROSS-PRODUCT";
        case PhotostationPlacerOptimizationErrorType::RAY_DIFF:
            return "RAY DIFF";
    }
    CORE_ASSERT_FALSE_S(true);
}

int corecvs::PhotostationPlacer::getErrorComponentsPerPoint()
{
    switch(errorType)
    {
        case PhotostationPlacerOptimizationErrorType::ANGULAR:
            return 1;
        case PhotostationPlacerOptimizationErrorType::REPROJECTION:
            return 2;
        case PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT:
            return 3;
        case PhotostationPlacerOptimizationErrorType::RAY_DIFF:
            return 3;
    }
    CORE_ASSERT_TRUE_S(false);
}

int corecvs::PhotostationPlacer::getReprojectionCnt()
{
    int total = 0;
    for (auto& o: tracks)
        total += (int)o.projections.size();
    return total * getErrorComponentsPerPoint();
}

int corecvs::PhotostationPlacer::getMovablePointCount()
{
    int movables = 0;
    for (auto& o: tracks)
        if (o.updateable)
            movables++;
    return movables;
}

int corecvs::PhotostationPlacer::getOrientationInputNum()
{
    int inputNum = 0;
    IF(DEGENERATE_ORIENTATIONS,
        inputNum += 4);
    IF(NON_DEGENERATE_ORIENTATIONS,
        inputNum += (preplaced - 1) * 4);
    IF(DEGENERATE_TRANSLATIONS,
        inputNum += 3);
    IF(NON_DEGENERATE_TRANSLATIONS,
        inputNum += (preplaced - 1) * 3);
    IF(FOCALS,
        inputNum += (int)calibratedPhotostations[0].cameras.size());
    IF(PRINCIPALS,
        inputNum += (int)calibratedPhotostations[0].cameras.size() * 2);
    IF(POINTS,
        inputNum += getMovablePointCount() * 3);
    return inputNum;
}

std::vector<std::vector<int>> corecvs::PhotostationPlacer::getDependencyList()
{
    revDependency.clear();
    int cnt = getReprojectionCnt();
    revDependency.resize(cnt);
    int argin = 0, id = 0;

    int errSize = getErrorComponentsPerPoint();

    for (int i = 0; i < (int)tracks.size(); ++i)
    {
        auto& t = tracks[i];
        for (int j = 0; j < (int)t.projections.size(); ++j)
        {
            for (int k = 0; k < errSize; ++k)
            {
                revDependency[id++] = std::make_pair(i, j);
            }
        }
    }
    CORE_ASSERT_TRUE_S(cnt == id);
    std::vector<std::vector<int>> sparsity(getOrientationInputNum());

    IF(DEGENERATE_ORIENTATIONS,
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < cnt; ++j)
            {
                auto p = revDependency[j];
                if (tracks[p.first].projections[p.second].photostationId == 0)
                    sparsity[argin].push_back(j);
            }
            ++argin;
        }
    );
    IF(NON_DEGENERATE_ORIENTATIONS,
        for (int i = 1; i < preplaced; ++i)
        {
            for (int jj = 0; jj < 4; ++jj)
            {
                for (int j = 0; j < cnt; ++j)
                {
                    auto p = revDependency[j];
                    if (tracks[p.first].projections[p.second].photostationId == i)
                        sparsity[argin].push_back(j);
                }
                ++argin;
            }
        });
    IF(DEGENERATE_TRANSLATIONS,
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < cnt; ++j)
            {
                auto p = revDependency[j];
                if (tracks[p.first].projections[p.second].photostationId == 0)
                    sparsity[argin].push_back(j);
            }
            ++argin;
        });
    IF(NON_DEGENERATE_TRANSLATIONS,
        for (int i = 1; i < preplaced; ++i)
        {
            for (int jj = 0; jj < 3; ++jj)
            {
                for (int j = 0; j < cnt; ++j)
                {
                    auto p = revDependency[j];
                    if (tracks[p.first].projections[p.second].photostationId == i)
                        sparsity[argin].push_back(j);
                }
                ++argin;
            }
        });
    IF(FOCALS,
        for (int i = 0; i < (int)calibratedPhotostations[0].cameras.size(); ++i)
        {
            for (int j = 0; j < cnt; ++j)
            {
                auto p = revDependency[j];
                if (tracks[p.first].projections[p.second].cameraId == i)
                    sparsity[argin].push_back(j);
            }
            ++argin;
        });
    IF(PRINCIPALS,
        for (int i = 0; i < (int)calibratedPhotostations[0].cameras.size(); ++i)
        {
            for (int jj = 0; jj < 2; ++jj)
            {
                for (int j = 0; j < cnt; ++j)
                {
                    auto p = revDependency[j];
                    if (tracks[p.first].projections[p.second].cameraId == i)
                        sparsity[argin].push_back(j);
                }
                ++argin;
            }
        });
    IF(POINTS,
        for (int i = 0; i < (int)tracks.size(); ++i)
        {
            for (int jj = 0; jj < 3; ++jj)
            {
                for (int j = 0; j < cnt; ++j)
                {
                    auto p = revDependency[j];
                    if (p.first == i)//].projections[p.second].cameraId == i)
                        sparsity[argin].push_back(j);
                }
                ++argin;
            }
        });
//    CORE_ASSERT_TRUE_S(getOrientationInputNum() == argin);
    return sparsity;   
}
    
void corecvs::PhotostationPlacer::readOrientationParams(const double in[])
{
    int argin = 0;
    IF(DEGENERATE_ORIENTATIONS,
        for (int i = 0; i < 4; ++i)
            GETPARAM(calibratedPhotostations[0].location.rotor[i])
        calibratedPhotostations[0].location.rotor.normalise();
    );
    IF(NON_DEGENERATE_ORIENTATIONS,
        for (int i = 1; i < preplaced; ++i)
        {
            for (int j = 0; j < 4; ++j)
                GETPARAM(calibratedPhotostations[i].location.rotor[j]);
            calibratedPhotostations[i].location.rotor.normalise();
        });
    IF(DEGENERATE_TRANSLATIONS,
        for (int i = 0; i < 3; ++i)
            GETPARAM(calibratedPhotostations[0].location.shift[i]));
    IF(NON_DEGENERATE_TRANSLATIONS,
        for (int i = 1; i < preplaced; ++i)
        {
            for (int j = 0; j < 3; ++j)
                GETPARAM(calibratedPhotostations[i].location.shift[j]);
        });
    IF(FOCALS,
        for (size_t i = 0; i < calibratedPhotostations[0].cameras.size(); ++i)
        {
            double f;
            GETPARAM(f);
            for (size_t j = 0; j < calibratedPhotostations.size(); ++j)
                calibratedPhotostations[j].cameras[i].intrinsics.focal = corecvs::Vector2dd(f, f);
                
        });
    IF(PRINCIPALS,
        for (size_t i = 0; i < calibratedPhotostations[0].cameras.size(); ++i)
        {
            double cx;
            double cy;
            GETPARAM(cx);
            GETPARAM(cy);
            for (size_t j = 0; j < calibratedPhotostations.size(); ++j)
                calibratedPhotostations[j].cameras[i].intrinsics.principal = corecvs::Vector2dd(cx, cy);
        });
    IF(POINTS,
        for (size_t j = 0; j < tracks.size(); ++j)
        {
            for (int i = 0; i < 3; ++i)
                GETPARAM(tracks[j].worldPoint[i]);
        });
//    CORE_ASSERT_TRUE_S(getOrientationInputNum() == argin);
}

void corecvs::PhotostationPlacer::writeOrientationParams(double out[])
{
    int argout = 0;
    IF(DEGENERATE_ORIENTATIONS,
        for (int i = 0; i < 4; ++i)
            SETPARAM(calibratedPhotostations[0].location.rotor[i]));
    IF(NON_DEGENERATE_ORIENTATIONS,
        for (int i = 1; i < preplaced; ++i)
        {
            for (int j = 0; j < 4; ++j)
                SETPARAM(calibratedPhotostations[i].location.rotor[j]);
        });
    IF(DEGENERATE_TRANSLATIONS,
        for (int i = 0; i < 3; ++i)
            SETPARAM(calibratedPhotostations[0].location.shift[i]));
    IF(NON_DEGENERATE_TRANSLATIONS,
        for (int i = 1; i < preplaced; ++i)
        {
            for (int j = 0; j < 3; ++j)
                SETPARAM(calibratedPhotostations[i].location.shift[j]);
        });
    IF(FOCALS,
        for (size_t i = 0; i < calibratedPhotostations[0].cameras.size(); ++i)
        {
            SETPARAM(calibratedPhotostations[0].cameras[i].intrinsics.focal[0]); 
        });
    IF(PRINCIPALS,
        for (size_t i = 0; i < calibratedPhotostations[0].cameras.size(); ++i)
        {
            SETPARAM(calibratedPhotostations[0].cameras[i].intrinsics.principal[0]);
            SETPARAM(calibratedPhotostations[0].cameras[i].intrinsics.principal[1]);
        });
    IF(POINTS,
        for (size_t j = 0; j < tracks.size(); ++j)
        {
            for (int i = 0; i < 3; ++i)
                SETPARAM(tracks[j].worldPoint[i]);
        });
//    CORE_ASSERT_TRUE_S(getOrientationInputNum() == argout);
}

void corecvs::PhotostationPlacer::fit(bool tuneFocal)
{
    fit(tuneFocal ? PhotostationPlacerOptimizationType::FOCALS | optimizationParams : optimizationParams);
}

void corecvs::PhotostationPlacer::getErrorSummaryAll()
{
    getErrorSummary(PhotostationPlacerOptimizationErrorType::REPROJECTION);
    getErrorSummary(PhotostationPlacerOptimizationErrorType::ANGULAR);
    getErrorSummary(PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT);
    getErrorSummary(PhotostationPlacerOptimizationErrorType::RAY_DIFF);
}

void corecvs::PhotostationPlacer::getErrorSummary(PhotostationPlacerOptimizationErrorType errorType)
{
    double totalsqr = 0.0;
    double totalcnt = 0.0;
    Vector3dd rE, rO, errV;
    double a;
    Vector2dd errP;
    for (auto& t: tracks)
        for (auto &p : t.projections)
        {
            totalcnt++;
            switch(errorType)
            {
                case PhotostationPlacerOptimizationErrorType::REPROJECTION:
                    errP = p.projection - calibratedPhotostations[p.photostationId].project(t.worldPoint, p.cameraId);
                    totalsqr += (!errP) * (!errP);
                    break;
                case PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT:
                    rE = calibratedPhotostations[p.photostationId].dirFromPixel(p.projection, p.cameraId);
                    rO = calibratedPhotostations[p.photostationId].dirFromPixel(calibratedPhotostations[p.photostationId].project(t.worldPoint, p.cameraId), p.cameraId);
                    errV = rE ^ rO;
                    totalsqr += !errV * !errV;
                    break;
                case PhotostationPlacerOptimizationErrorType::ANGULAR:
                    rE = calibratedPhotostations[p.photostationId].dirFromPixel(p.projection, p.cameraId);
                    rO = calibratedPhotostations[p.photostationId].dirFromPixel(calibratedPhotostations[p.photostationId].project(t.worldPoint, p.cameraId), p.cameraId);
                    a = rE.angleTo(rO) * 180.0 / M_PI;
                    totalsqr += a * a;
                    break;
                case PhotostationPlacerOptimizationErrorType::RAY_DIFF:
                    rE = calibratedPhotostations[p.photostationId].dirFromPixel(p.projection, p.cameraId);
                    rO = calibratedPhotostations[p.photostationId].dirFromPixel(calibratedPhotostations[p.photostationId].project(t.worldPoint, p.cameraId), p.cameraId);
                    errV = rE - rO;
                    totalsqr += !errV * !errV;
                    break;
            }
        }

    std::cout << toString(errorType) << " RMSE: " << std::sqrt(totalsqr / totalcnt) << std::endl;
}

void corecvs::PhotostationPlacer::fit(const PhotostationPlacerOptimizationType &params, int num)
{
    getErrorSummaryAll();
    optimizationParams = params;
    corecvs::LevenbergMarquardtSparse lm;
    OrientationFunctor orient(this);
    OrientationNormalizationFunctor orientNorm(this);
    lm.f = &orient;
    lm.normalisation = &orientNorm;
    lm.maxIterations = num;
    lm.trace = false;
    std::vector<double> input(getOrientationInputNum());
    std::vector<double> out(getReprojectionCnt());
    lm.useConjugatedGradient = false;
    lm.conjugatedGradientIterations = std::max(100, (int)( 0.001 * input.size()));
    writeOrientationParams(&input[0]);
    auto res = lm.fit(input, out);
    readOrientationParams(&res[0]);

    getErrorSummaryAll();
    placed = preplaced;
}

void corecvs::PhotostationPlacer::ParallelErrorComputator::operator() (const corecvs::BlockedRange<int> &r) const
{
    auto& tracks = placer->tracks;
    auto& revDependency = placer->revDependency;
    auto& calibratedPhotostations = placer->calibratedPhotostations;
    switch(placer->errorType)
    {
        case PhotostationPlacerOptimizationErrorType::REPROJECTION:
            for (int ii = r.begin(); ii < r.end(); ii++)
            {
                int i = idxs[ii * 2];   
                CORE_ASSERT_TRUE_S(idxs[ii * 2] + 1 == idxs[ii * 2 + 1]);
                auto& o = tracks[revDependency[i].first];
                auto& p = o.projections[revDependency[i].second];
                auto error = p.projection - calibratedPhotostations[p.photostationId].project(o.worldPoint, p.cameraId);
                output[ii * 2]     = error[0];
                output[ii * 2 + 1] = error[1];
            }
            break;
        case PhotostationPlacerOptimizationErrorType::ANGULAR: 
            for (int ii = r.begin(); ii < r.end(); ++ii)
            {
                int i = idxs[ii];
                auto& o = tracks[revDependency[i].first];
                auto& p = o.projections[revDependency[i].second];
                auto cam = calibratedPhotostations[p.photostationId].getRawCamera(p.cameraId);
                auto dirO = cam.rayFromPixel(calibratedPhotostations[p.photostationId].project(o.worldPoint, p.cameraId)).a.normalised();
                auto dirE = cam.rayFromPixel(p.projection).a.normalised();
                double a = dirO.angleTo(dirE);
                output[ii] = a * M_PI / 180.0;
            }
            break;
        case PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT: 
            for (int ii = r.begin(); ii < r.end(); ++ii)
            {
                int i = idxs[ii * 3];
                CORE_ASSERT_TRUE_S(idxs[ii * 3] + 1 == idxs[ii * 3 + 1] && idxs[ii * 3] + 2 == idxs[ii * 3 + 2]);
                auto& o = tracks[revDependency[i].first];
                auto& p = o.projections[revDependency[i].second];
                auto cam = calibratedPhotostations[p.photostationId].getRawCamera(p.cameraId);
                auto dirO = cam.rayFromPixel(calibratedPhotostations[p.photostationId].project(o.worldPoint, p.cameraId)).a.normalised();
                auto dirE = cam.rayFromPixel(p.projection).a.normalised();
                auto a = dirO ^ dirE;
                for (int j = 0; j < 3; ++j)
                    output[ii * 3 + j] = a[j];
            }
            break;
        case PhotostationPlacerOptimizationErrorType::RAY_DIFF: 
            for (int ii = r.begin(); ii < r.end(); ++ii)
            {
                int i = idxs[ii * 3];
                CORE_ASSERT_TRUE_S(idxs[ii * 3] + 1 == idxs[ii * 3 + 1] && idxs[ii * 3] + 2 == idxs[ii * 3 + 2]);
                auto& o = tracks[revDependency[i].first];
                auto& p = o.projections[revDependency[i].second];
                auto cam = calibratedPhotostations[p.photostationId].getRawCamera(p.cameraId);
                auto dirO = cam.rayFromPixel(calibratedPhotostations[p.photostationId].project(o.worldPoint, p.cameraId)).a.normalised();
                auto dirE = cam.rayFromPixel(p.projection).a.normalised();
                auto a = dirO - dirE;
                for (int j = 0; j < 3; ++j)
                    output[ii * 3 + j] = a[j];
            }
            break;
    }
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
    ParallelErrorComputator computator(this, idxs, out);
    corecvs::parallelable_for(0, (int)idxs.size() / getErrorComponentsPerPoint(), 16, computator);
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
            if (inlierIdx >= (int)inlierIds.size() || totalIdx++ != inlierIds[inlierIdx])
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

#if 0
corecvs::Vector3dd corecvs::PhotostationPlacer::comp
#endif
void corecvs::PhotostationPlacer::appendPs()
{
    int psApp = placed;
    std::cout << "Placing #" << placed << std::endl;
    auto hypos = getPossibleTracks(psApp);
    std::cout << "Total " << hypos.size() << " possible tracks" << std::endl;

//    AbsoluteNonCentralRansacSolver solver(calibratedPhotostations[psApp], hypos);
    corecvs::AbsoluteNonCentralRansacSolverParams params;
    switch(psInitData[psApp].initializationType)
    {
        case PhotostationInitializationType::GPS:
            {
                params.forcePosition = true;
                params.forcedPosition = psInitData[psApp].gpsData;
                AbsoluteNonCentralRansacSolver solver(calibratedPhotostations[psApp], hypos, params);
                solver.run();
                solver.runInliersRE();
                auto res = solver.getInliers();
                auto hypo = solver.getBestHypothesis();
                std::cout << "PS append: " << res.size() << std::endl;
                std::cout << hypo.shift << " " << hypo.rotor << " | " << psInitData[psApp].gpsData << std::endl;
                calibratedPhotostations[psApp].location = hypo;
                calibratedPhotostations[psApp].location.shift = psInitData[psApp].gpsData;
                preplaced++;
                fit(optimizationParams, 100);
                appendTracks(res, psApp);
            }
            break;
        case PhotostationInitializationType::NONE:
            {
                corecvs::AbsoluteNonCentralRansacSolverParams params;
                params.forcePosition = false;
                AbsoluteNonCentralRansacSolver solver(calibratedPhotostations[psApp], hypos, params);
                solver.run();
                solver.runInliersRE();
                auto res = solver.getInliers();
                auto hypo = solver.getBestHypothesis();
                std::cout << "PS append: " << res.size() << std::endl;
                std::cout << hypo.shift << " " << hypo.rotor << std::endl;
                calibratedPhotostations[psApp].location = hypo;
                appendTracks(res, psApp);
            }
            break;
        case PhotostationInitializationType::STATIC_POINTS:
            {
                auto &ps = calibratedPhotostations[psApp];
                ps.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
                ps.location.shift = corecvs::Vector3dd(0, 0, 0);
	            std::vector<corecvs::Vector3dd> centers, directions, points3d;
                for (auto& t: hypos)
                {
                    int cam = std::get<0>(t);
                    auto pt = std::get<1>(t);
                    auto ptw= std::get<3>(t);

                    centers.push_back(ps.getRawCamera(cam).extrinsics.position);
                    directions.push_back(ps.getRawCamera(cam).rayFromPixel(pt).a);
                    points3d.push_back(ptw);
                }

                auto hypo = corecvs::PNPSolver::solvePNP(centers, directions, points3d);
                auto bestHypo = hypo[0];
                int bestCnt = 0;
                std::vector<int> bestInliers = {};
                for (auto &h: hypo)
                {
                    ps.location = h;
                    int cnt = 0;
                    std::vector<int> inliers;
                    for (auto& cp: hypos)
                    {
                        auto pp = ps.project(std::get<3>(cp), std::get<0>(cp)) - std::get<1>(cp);
                        if (!pp < 2)
                        {
                            cnt++;
                            inliers.push_back(&cp - &*hypos.begin());
                        }
                    }
                    if (cnt > bestCnt)
                    {
                        bestCnt = cnt;
                        bestInliers = inliers;
                        bestHypo = h;
                    }
                }
                ps.location = bestHypo;
                appendTracks(bestInliers, psApp);
            }
            break;
        default:
            CORE_ASSERT_TRUE_S(false);
    }
    return;
}

#if 0
void corecvs::PhotostationPlacer::placeWithStaticPoints(int psId)
{
    auto& sp = psInitData[psId].staticPoints;
    CORE_ASSERT_TRUE_S(sp.size() >= 3);
}
#endif

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
        for (int camB = 0; camB < (int)calibratedPhotostations[psB].cameras.size(); ++camB)
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
            int id = (int)tracks.size();
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


#if 0
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
#endif

void corecvs::PhotostationPlacer::selectEpipolarInliers(int psA, int psB)
{
    for (int camA = 0; camA < (int)calibratedPhotostations[psA].cameras.size(); ++camA)
    {
        for (int camB = 0; camB < (int)calibratedPhotostations[psB].cameras.size(); ++camB)
        {
            corecvs::Matrix33 F = calibratedPhotostations[psA].getRawCamera(camA).fundamentalTo(calibratedPhotostations[psB].getRawCamera(camB));
            corecvs::EssentialDecomposition E = calibratedPhotostations[psA].getRawCamera(camA).essentialDecomposition(calibratedPhotostations[psB].getRawCamera(camB));
            auto K1 = calibratedPhotostations[psA].getRawCamera(camA).intrinsics.getKMatrix33().inv();
            auto K2 = calibratedPhotostations[psB].getRawCamera(camB).intrinsics.getKMatrix33().inv();
            auto mc = getCameraMatches(psA, camA, psB, camB);
            std::vector<int> outliers;

            for (size_t i = 0; i < mc.size(); ++i)
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
                    outliers.push_back((int)i);
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

std::atomic<int> corecvs::PhotostationPlacer::ParallelEssentialFilter::cntr;
void corecvs::PhotostationPlacer::estimateFirstPair()
{
#ifdef WITH_TBB
    tbb::task_group g;
    g.run([=]() { estimatePair(0, 1); });
    g.run([=]() { estimatePair(0, 2); });
    g.wait();
#else
    estimatePair(0, 1);
    estimatePair(0, 2);
#endif

    dumpMesh("triples_before_reorient.ply");

    auto q = detectOrientationFirst();
    for (int psA = 0; psA < 3; ++psA)
        for (int psB = 0; psB < 3; ++psB)
    std::cout << calibratedPhotostations[psA].getRawCamera(0).rayFromPixel(corecvs::Vector2dd(0, 0)).a.angleTo(calibratedPhotostations[psB].location.shift - calibratedPhotostations[psA].location.shift) << std::endl;
    calibratedPhotostations[0].location.rotor = q.conjugated();
    calibratedPhotostations[0].location.shift = psInitData[0].gpsData;
    calibratedPhotostations[1].location.rotor = q.conjugated() ^ calibratedPhotostations[1].location.rotor;
    calibratedPhotostations[1].location.shift = psInitData[1].gpsData;
    calibratedPhotostations[2].location.rotor = q.conjugated() ^ calibratedPhotostations[2].location.rotor;
    calibratedPhotostations[2].location.shift = psInitData[2].gpsData;
    for (int psA = 0; psA < 3; ++psA)
        for (int psB = 0; psB < 3; ++psB)
    std::cout << calibratedPhotostations[psA].getRawCamera(0).rayFromPixel(corecvs::Vector2dd(0, 0)).a.angleTo(calibratedPhotostations[psB].location.shift - calibratedPhotostations[psA].location.shift) << std::endl;

    preplaced = 3;
    matches = matchesCopy;
}

corecvs::Quaternion corecvs::PhotostationPlacer::detectOrientationFirst()
{
    std::cout << "ABC: " << (psInitData[1].gpsData - psInitData[0].gpsData).angleTo(psInitData[2].gpsData - psInitData[0].gpsData) << " | " << calibratedPhotostations[1].location.shift.angleTo(calibratedPhotostations[2].location.shift) << std::endl;

    corecvs::Vector3dd e1 = psInitData[1].gpsData - psInitData[0].gpsData;
    corecvs::Vector3dd e2 = psInitData[2].gpsData - psInitData[0].gpsData;

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
    corecvs::Matrix33 Vt;
    corecvs::Matrix::svd(&RC, &v, &Vt);

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
#if 0
    pairInliers.emplace_back(psA, psB, solver.getBestInliers());
#endif
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
    int N = (int)ransacableIdx.size();
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
    for (int i = 0; i < (int)matches.size(); ++i)
    {
        if (inlierId < (int)bestInliers.size() && bestInliers[inlierId] == i)
        {
            inlierId++;
            continue;
        }
        delIdx.push_back(i);
    }
#ifdef WITH_TBB
    tbb::mutex::scoped_lock(mutex);
#endif
    remove(psA, camA, psB, camB, delIdx);
}

void corecvs::PhotostationPlacer::filterEssentialRansac()
{
    CORE_ASSERT_TRUE_S(calibratedPhotostations.size() >= 3);
    matchesCopy = matches;
    std::vector<std::tuple<int, int, int, int>> work;
    for (int psA = 0; psA < 3; ++psA)
    {
        for (int psB = psA; psB < 3; ++psB)
        {
            for (int camA = 0; camA < (int)calibratedPhotostations[psA].cameras.size(); ++camA)
            {
                for (int camB = 0; camB < (int)calibratedPhotostations[psB].cameras.size(); ++camB)
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
    for (int i = 0; i < (int)matches[id1][id2 - id1].size(); ++i)
    {
        if (idxSkip < (int)idx.size() && i == idx[idxSkip])
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
    for (size_t i = 0; i < psps.size(); ++i)
    {
        auto t = psps[i];
        int cam1 = std::get<0>(t);
        int cam2 = std::get<2>(t);
        if (cam1 != camA || cam2 != camB)
            continue;
        if (idxSkip < (int)idx.size() && idx[idxSkip] == idxCams)
        {
            idxSkip++;
            skipGlobal.push_back((int)i);
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
    for (size_t i = 0; i < matches[id1][id2 - id1].size(); ++i)
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
	for (int i = 0; i < (int)calibratedPhotostations.size(); ++i)
	{
		for (int j = 0; j < (int)images[i].size(); ++j)
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
#endif
void corecvs::PhotostationPlacer::detectAll()
{
#if 0
    std::vector<std::string> filenames;
    std::unordered_map<int, std::pair<int, int>> img_map;
    for (int i = 0; i < (int)images.size(); ++i)
    {
        for (int j = 0; j < (int)images[i].size(); ++j)
        {
            filenames.push_back(images[i][j]);
            img_map[(int)filenames.size() - 1] = std::make_pair(i, j);
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

    for (size_t i = 0; i < keyPoints.size(); ++i)
    {
        keyPoints[i].resize(calibratedPhotostations[i].cameras.size());
        keyPointColors[i].resize(calibratedPhotostations[i].cameras.size());
    }
    int N = (int)filenames.size();
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
    for (size_t i = 0; i < calibratedPhotostations.size(); ++i)
        matches[i].resize(calibratedPhotostations.size() - i);

    auto& ref = pipeline.refinedMatches.matchSets;
    for (auto& ms: ref)
    {
        auto id1 = img_map[(int)ms.imgA];
        auto id2 = img_map[(int)ms.imgB];
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
#else
    scene->detectAllFeatures(FeatureDetectionParams());
#endif
}

#if 0
std::vector<PointObservation__> corecvs::PhotostationPlacer::projectToAll(const std::vector<PointObservation__> &pois)
{
    auto ret = pois;
    for (auto& o: ret)
    {
        for (int ps = 0; ps < (int)calibratedPhotostations.size(); ++ps)
        {
            for (int cam = 0; cam < (int)calibratedPhotostations[ps].cameras.size(); ++cam)
            {
                if (calibratedPhotostations[ps].isVisible(o.worldPoint, cam))
                {
                    PointProjection proj;
                    proj.photostationId = ps;
                    proj.cameraId = cam;
                    proj.projection = calibratedPhotostations[ps].project(o.worldPoint, cam);
                    o.projections.push_back(proj);
                }
            }
        }
    }
    return ret;
}

void corecvs::PhotostationPlacer::fullRun()
{
    detectAll();
    filterEssentialRansac();
    estimateFirstPair();

    std::cout << "PRETRACK" << std::endl;
    buildTracks(0, 1, 2);
    std::cout << "POSTTRACK1" << std::endl;
	fit();
    std::cout << "POSTFIT1" << std::endl;
	buildTracks(0, 1, 2);
    std::cout << "POSTTRACK2" << std::endl;

	for (int i = 3; i < (int)calibratedPhotostations.size(); ++i)
	{

        appendPs();
        for (int j = 0; j < i; ++j)
            for (int k = j + 1; k < i; ++k)
                buildTracks(j, k, i);
	}
	fit(PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS, 10000);
}

corecvs::Mesh3D corecvs::PhotostationPlacer::dumpMesh(const std::string &filename, bool drawTracks, bool center)
{
    corecvs::Mesh3D meshres;
    meshres.switchColor(true);
    corecvs::Vector3dd meanpos(0, 0, 0);
    if (center)
    {
        for (int i = 0; i < placed; ++i)
        {
            meanpos += calibratedPhotostations[i].location.shift * (1.0 / placed);
        }
    }
    for (int i = 0; i < placed; ++i)
    {
        corecvs::Photostation ps = calibratedPhotostations[i];
        for (size_t j = 0; j < ps.cameras.size(); ++j)
            ps.cameras[j].extrinsics.position *= 1e3;
        ps.location.shift -= meanpos;
        ps.location.shift *= 1e3;

        CalibrationHelpers().drawPly(meshres, ps, 50.0);
    }
    if (drawTracks)
    {
        size_t projs = 0;
        std::map<int, int> cntp;
        std::cout << "TRACKS: " << tracks.size() << std::endl;
        for(auto&p : tracks)
        {
            cntp[p.projections.size()]++;
            projs += p.projections.size();
            auto proj = p.projections[0];
            auto col = keyPointColors[proj.photostationId][proj.cameraId][proj.featureId];
            meshres.setColor(col);
            meshres.addPoint((p.worldPoint-meanpos) * 1e3);
        }
        std::cout << "Total " << projs << " projections" << std::endl;
        for (auto& p : cntp)
            std::cout << p.first << ": " << p.second << std::endl;
    }
    meshres.dumpPLY(filename);
    return meshres;
}
#endif
