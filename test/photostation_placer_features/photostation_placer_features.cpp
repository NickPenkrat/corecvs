/*
 * This test takes photostation model from calibrationJob and arranges copies
 * of this PS using GPS coordinates
 *
 * NOTE: this one is not finalized yet and is more "playground" than "application"
 */
#include <vector>
#include <string>
#include <sstream>
#include <regex>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <cstdio>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <random>

#include "calibrationHelpers.h"
#include "calibrationJob.h"
#include "calibrationLocation.h"
#include "mesh3d.h"
#include "jsonSetter.h"
#include "jsonGetter.h"

#include "imageKeyPoints.h"
#include "reconstructionStructs.h"

#include "vector3d.h"
#include "vector2d.h"
#include "bufferReaderProvider.h"
#include "tbb/task_group.h"

#include "openCvFileReader.h"
#include "featureMatchingPipeline.h"
#include "openCvDescriptorExtractorWrapper.h"
#include "openCvFeatureDetectorWrapper.h"
#include "openCvDescriptorMatcherWrapper.h"
#include "multicameraTriangulator.h"
#include "undirectedGraph.h"
#include "multiPhotostationScene.h"
#include "essentialEstimator.h"

#include "photostationPlacer.h"

#if 1
int main()
{
    init_opencv_detectors_provider();
    init_opencv_matchers_provider();
    init_opencv_reader_provider();
    init_opencv_descriptors_provider();
    init_opencv_reader_provider();

    corecvs::Vector3dd positions[] =
    {
        corecvs::Vector3dd(140.617, 576.200, 164.136),
        corecvs::Vector3dd(134.637, 575.080, 164.099),
        corecvs::Vector3dd(139.161, 572.191, 164.056),
        corecvs::Vector3dd(132.418, 571.214, 164.020),
        corecvs::Vector3dd(138.423, 567.795, 164.064),
        corecvs::Vector3dd(129.873, 567.804, 164.097),
        corecvs::Vector3dd(134.916, 563.506, 164.114),
        corecvs::Vector3dd(124.671, 567.081, 164.121),
        corecvs::Vector3dd(146.231, 575.418, 164.191),
    };

    PhotostationPlacer pp;
    const int PSN = 9;
    const int CP = 6;
    char psPrefixes[9] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};
    
    std::string prefix = "roof_v1/roof_1_SP";
    std::string postfix= "_0deg_undist.jpg";

    pp.images.resize(PSN);
    for (int i = 0; i < PSN; ++i)
    {
        pp.images[i].resize(CP);
        for (int j = 0; j < CP; ++j)
        {
            std::stringstream ss;
            ss << prefix << psPrefixes[i] << j << postfix;
            pp.images[i][j] = ss.str();
        }
    }
    pp.gpsData.resize(PSN);
    for (int i = 0; i < PSN; ++i)
    {
        pp.gpsData[i] = positions[psPrefixes[i] - 'A'];
    }

    corecvs::Photostation photostation;
    JSONGetter getter("calibration.json");
    CalibrationJob job;
    getter.visit(job, "job");
    photostation = job.photostation;
    for (int i = 0; i < photostation.cameras.size(); ++i)
    {
        photostation.cameras[i].extrinsics.position /= 1e3;
    }

    pp.calibratedPhotostations.resize(PSN);
    for (int i = 0; i < PSN; ++i)
        pp.calibratedPhotostations[i] = photostation;

    pp.detectAll();
    pp.filterEssentialRansac();
    pp.estimateFirstPair();
    
    Mesh3D meshres;
    meshres.switchColor(true);
    corecvs::Vector3dd meanpos(0, 0, 0);
    for (int i = 0; i < PSN; ++i)
    {
        meanpos += pp.calibratedPhotostations[i].location.shift * (1.0 / PSN);
    }
    for (int i = 0; i < PSN; ++i)
    {
        std::stringstream ss;
        ss << "SP" << ((char)('A' + i));
        corecvs::Photostation ps = pp.calibratedPhotostations[i];
        for (int j = 0; j < ps.cameras.size(); ++j)
            ps.cameras[j].extrinsics.position *= 1e3;
        ps.location.shift -= meanpos;
        ps.location.shift *= 1e3;
        ps.name = ss.str();

        CalibrationHelpers().drawPly(meshres, ps, 50.0);
    }

    pp.selectEpipolarInliers();
    pp.backprojectAll();
    meshres.dumpPLY("triples.ply");
#if 0
    Mesh3D meshres2;
    meshres2.switchColor(true);
    meanpos = corecvs::Vector3dd(0, 0, 0);
    for (int i = 0; i < PSN; ++i)
    {
        meanpos += pp.calibratedPhotostations[i].location.shift * (1.0 / PSN);
    }
    for (int i = 0; i < PSN; ++i)
    {
        std::stringstream ss;
        ss << "SP" << ((char)('A' + i));
        corecvs::Photostation ps = pp.calibratedPhotostations[i];
        for (int j = 0; j < ps.cameras.size(); ++j)
            ps.cameras[j].extrinsics.position *= 1e3;
        ps.location.shift -= meanpos;
        ps.location.shift *= 1e3;
		std::cout << ps.location.shift << std::endl;
        ps.name = ss.str();

        CalibrationHelpers().drawPly(meshres2, ps, 50.0);
    }
//	std::cout << "BP size: " << pp.backprojected.size() << std::endl;
	int colors[] = { 0xffff00, 0xff00ff, 0x00ffff };
	for (int i = 0; i < 3; ++i)
	{
		meshres2.setColor(corecvs::RGBColor(colors[i]));
    for(auto&p : pp.backprojected[i])
        meshres2.addPoint((p.second-meanpos) * 1e3);
	}
    meshres2.dumpPLY("triples_bp.ply");
#endif
    pp.buildTracks(0, 1, 2);
	std::cout << "TR: " << pp.tracks.size() << " " << pp.getReprojectionCnt() / 2 << std::endl;
	pp.fitLMedians();
	pp.buildTracks(0, 1, 2);
	std::cout << "TR: " << pp.tracks.size() << " " << pp.getReprojectionCnt() / 2 << std::endl;

    meshres.dumpPLY("triples_tri.ply");
    Mesh3D meshres3;
    meshres3.switchColor(true);
    meanpos = corecvs::Vector3dd(0, 0, 0);
    for (int i = 0; i < PSN; ++i)
    {
        meanpos += pp.calibratedPhotostations[i].location.shift * (1.0 / PSN);
    }
    for (int i = 0; i < PSN; ++i)
    {
        std::stringstream ss;
        ss << "SP" << ((char)('A' + i));
        corecvs::Photostation ps = pp.calibratedPhotostations[i];
        for (int j = 0; j < ps.cameras.size(); ++j)
            ps.cameras[j].extrinsics.position *= 1e3;
        ps.location.shift -= meanpos;
        ps.location.shift *= 1e3;
		std::cout << ps.location.shift << std::endl;
        ps.name = ss.str();

        CalibrationHelpers().drawPly(meshres3, ps, 50.0);
    }
	std::cout << "TRACKS: " << pp.tracks.size() << std::endl;
    for(auto&p : pp.tracks)
        meshres3.addPoint((p.worldPoint-meanpos) * 1e3);
    meshres3.dumpPLY("triples_tri.ply");

	for (int i = 3; i < PSN; ++i)
	{
    pp.fitLMedians();
    pp.appendPs();
    for (int j = 0; j < i; ++j)
    	for (int k = j + 1; k < i; ++k)
    		pp.buildTracks(j, k, i);
	}

    Mesh3D meshres4;
    meshres4.switchColor(true);
    meanpos = corecvs::Vector3dd(0, 0, 0);
    for (int i = 0; i < PSN; ++i)
    {
        meanpos += pp.calibratedPhotostations[i].location.shift * (1.0 / PSN);
    }
    for (int i = 0; i < PSN; ++i)
    {
        std::stringstream ss;
        ss << "SP" << ((char)('A' + i));
        corecvs::Photostation ps = pp.calibratedPhotostations[i];
        for (int j = 0; j < ps.cameras.size(); ++j)
            ps.cameras[j].extrinsics.position *= 1e3;
        ps.location.shift -= meanpos;
        ps.location.shift *= 1e3;
		std::cout << ps.location.shift << std::endl;
        ps.name = ss.str();

        CalibrationHelpers().drawPly(meshres4, ps, 50.0);
    }
	std::cout << "TRACKS: " << pp.tracks.size() << std::endl;
	int projs = 0;
	std::unordered_map<int, int> cntp;
    for(auto&p : pp.tracks)
	{
		cntp[p.projections.size()]++;
		projs += p.projections.size();
		auto proj = p.projections[0];
		auto col = pp.keyPointColors[proj.photostationId][proj.cameraId][proj.featureId];
		meshres4.setColor(col);
        meshres4.addPoint((p.worldPoint-meanpos) * 1e3);
	}
	std::cout << "Total " << projs << " projections" << std::endl;
	for (auto& p : cntp)
		std::cout << p.first << ": " << p.second << std::endl;
    meshres4.dumpPLY("triples_app.ply");
    return 0;
}
#else
corecvs::Vector2dd transform(corecvs::Matrix33 iK, corecvs::Vector2dd uv)
{
    corecvs::Vector3dd uv1 = corecvs::Vector3dd(uv[0], uv[1], 1.0);
    auto vv = iK * uv1;
    vv /= vv[2];
    return corecvs::Vector2dd(vv[0], vv[1]);
}

const size_t CAM_PER_PS = 6;
const std::string descriptor = "SURF";
const std::string detector   = "SURF";
const std::string matcher    = "ANN";

double angleThreshold = 5.0;
double pxSimilarityRansac = 2.0;
double pxSimilarityCombine = 16.0;
double ransacB2B = 0.8;
double inlierB2B = 0.9;
long long ransacIterationLimit = 10000;

std::string filenamesUndist[3][CAM_PER_PS];
std::vector<KeyPoint> keypoints[3][CAM_PER_PS];
// camRef camQuery KeyPointRef -> KeyPointQuery
std::map<std::tuple<int, int, int>, int> triLookup[3];
int map_tri[3][3] = {{-1, 0, 2}, {-1, -1, 1}, {-1, -1, -1}};

corecvs::Quaternion QQ [2][CAM_PER_PS * CAM_PER_PS];
corecvs::Vector3dd  tt [2][CAM_PER_PS * CAM_PER_PS];
size_t              cnt[2][CAM_PER_PS * CAM_PER_PS];
size_t             fcnt[6][CAM_PER_PS * CAM_PER_PS];
int map_fcnt[3][3] = {{0, 1, 2}, {1, 3, 4}, {2, 4, 5}};
std::vector<corecvs::Photostation> pss(3);

corecvs::Quaternion Q [2];    // Final quaternion

corecvs::Vector3dd  T [2];

std::unordered_map<size_t, std::pair<size_t, size_t>> cam_map;
std::vector<std::tuple<double, int, int, int>> angleBAC;
RefinedMatches refined;

corecvs::Photostation photostation;

corecvs::Vector3dd positions[] =
{
    corecvs::Vector3dd(140.617, 576.200, 164.136) * 1e3,
    corecvs::Vector3dd(134.637, 575.080, 164.099) * 1e3,
    corecvs::Vector3dd(139.161, 572.191, 164.056) * 1e3
};

void computeAngles()
{
    angleBAC.clear();
    for (int i = 0; i < CAM_PER_PS * CAM_PER_PS; ++i)
    {
        for (int j = 0; j < CAM_PER_PS * CAM_PER_PS; ++j)
        {
            double angle = tt[0][i].angleTo(tt[1][j]);
            int inliers = cnt[0][i] + cnt[1][j];
            angleBAC.emplace_back(angle, inliers, i, j);
        }
    }
    std::sort(angleBAC.begin(), angleBAC.end(), [](const std::tuple<double, int, int, int> &a, const std::tuple<double, int, int, int> &b) { return std::get<1>(a) > std::get<1>(b); });
    std::cout << "Real angle: " << (positions[1] - positions[0]).angleTo(positions[2] - positions[0]) *180.0/M_PI << std::endl;
    std::cout << "Top " << 100 << " hypos:" << std::endl;
    for (int i = 0; i < 100; ++i)
    {
        auto t = angleBAC[i];
        std::cout << std::get<0>(t)*180.0/M_PI << " degrees (" << std::get<1>(t) << ") using " << (std::get<2>(t) / CAM_PER_PS) << "->" << (std::get<2>(t) % CAM_PER_PS) << " and " << (std::get<3>(t) / CAM_PER_PS) << "->" << (std::get<3>(t) % CAM_PER_PS) << std::endl;
    }
    int incnt = 0;
    for (auto&h : angleBAC)
        if (std::abs(std::get<0>(h) - (positions[1] - positions[0]).angleTo(positions[2] - positions[0])) < 5 * M_PI / 180.0)
            incnt++;
    std::cout << "Total " << incnt << " hypothesis inside 5deg" << std::endl;
}

corecvs::Quaternion fit(corecvs::Vector3dd t1, corecvs::Vector3dd t2, corecvs::Vector3dd b1, corecvs::Vector3dd b2)
{
    corecvs::Matrix A(9, 9);
    corecvs::Vector B(9);
    corecvs::Matrix33 RC;
    t1 = t1.normalised();
    t2 = t2.normalised();
    b1 = b1.normalised();
    b2 = b2.normalised();
    auto t3 = t1 ^ t2;
    auto b3 = b1 ^ b2;
    for (int i = 0; i < 3; ++i)
    {
        A.a(0, i) = A.a(1, i + 3) = A.a(2, i + 6) = t1[i];
        A.a(3, i) = A.a(4, i + 3) = A.a(5, i + 6) = t2[i];
        A.a(6, i) = A.a(7, i + 3) = A.a(8, i + 6) = t3[i];
        B[i] = b1[i];
        B[i + 3] = b2[i];
        B[i + 6] = b3[i];
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

int createTriples(size_t id1, size_t id2)
{
    auto QPB = QQ[0][id1];
    auto QPC = QQ[1][id2];
    auto b1 =  positions[1] - positions[0];
    auto b2 =  positions[2] - positions[0];
    auto t1 =  tt[0][id1];
    auto t2 =  tt[1][id2];
    auto Q = fit(t1, t2, b1, b2);
    pss[0] = pss[1] = pss[2] = photostation;
    pss[0].location.shift = positions[0];
    pss[1].location.shift = positions[1];
    pss[2].location.shift = positions[2];
    pss[0].location.rotor = Q;//tions[0];
    pss[1].location.rotor = Q^QPB;//s[2];
    pss[2].location.rotor = Q^QPC;//s[3];

    b1.normalise();
    b2.normalise();
    t1.normalise();
    t2.normalise();

    for (int i = 0; i < 3; ++i)
        triLookup[i].clear();

    int inliers = 0;
    for (auto& ms: refined.matchSets)
    {
        int imgA = ms.imgA;
        int imgB = ms.imgB;
        auto p1 = cam_map[imgA];
        auto p2 = cam_map[imgB];
        auto K1 = pss[p1.first].getRawCamera(p1.second).intrinsics.getKMatrix33().inv();
        auto K2 = pss[p2.first].getRawCamera(p2.second).intrinsics.getKMatrix33().inv();
        auto E = pss[p1.first].getRawCamera(p1.second).essentialDecomposition(pss[p2.first].getRawCamera(p2.second));
        auto FM = pss[p1.first].getRawCamera(p1.second).fundamentalTo(pss[p2.first].getRawCamera(p2.second));
        if (p1.first == p2.first)
            continue;
        bool swap = p1.first > p2.first;
        assert(std::min(p1.first, p2.first) < 2);
        int index = map_tri[std::min(p1.first, p2.first)][std::max(p1.first, p2.first)];
        assert(index >= 0 && index < 3);

        for (auto& m: ms.matches)
        {
            auto kpL = keypoints[p1.first][p1.second][m.featureA];
            auto kpR = keypoints[p2.first][p2.second][m.featureB];

            corecvs::Vector2dd L(kpL.x, kpL.y), R(kpR.x, kpR.y);
            corecvs::Vector3dd L3(L[0], L[1], 1.0), R3(R[0], R[1], 1.0);
            auto lineL = FM.mulBy2dRight(R);
            auto lineR = FM.mulBy2dLeft(L);
            double diff = std::abs(corecvs::Vector3dd(L[0], L[1], 1.0) & lineL);
            double lnorm = std::min(std::sqrt(lineL[0] * lineL[0] + lineL[1] * lineL[1]), std::sqrt(lineR[0] * lineR[0] + lineR[1] * lineR[1]));
            double scaleL, scaleR, foo;
            auto R1 = K2 * R3;
             R1 /= R1[2];
            auto L1 = K1 * L3;
                        L1 /= L1[2];
            E.getScaler(R1, L1, scaleL, scaleR, foo);
            if (scaleL > 0.0 && scaleR > 0.0 && diff / lnorm < pxSimilarityCombine)
            {
                std::tuple<int, int, int> key;
                int value;
                if (swap)
                {
                    std::get<0>(key) = p2.second;
                    std::get<1>(key) = p1.second;
                    std::get<2>(key) = m.featureB;
                    value = m.featureA;
                }
                else
                {
                    std::get<0>(key) = p1.second;
                    std::get<1>(key) = p2.second;
                    std::get<2>(key) = m.featureA;
                    value = m.featureB;
                }
                triLookup[index][key] = value;
            }
        }
    }
    std::vector<std::tuple<int, int, int, int, int, int>> triplets;
    for (auto& t1: triLookup[2])
    {
        auto kAC = t1.first;
        auto vAC = t1.second;
        for (int i = 0; i < CAM_PER_PS; ++i)
        {
            auto kAB = kAC;
            std::get<1>(kAB) = i;
            if (!triLookup[0].count(kAB))
                continue;
            auto kBC = kAB;
            std::get<0>(kBC) = i;
            std::get<1>(kBC) = std::get<1>(kAC);
            std::get<2>(kBC) = triLookup[0][kAB];
            if (!triLookup[1].count(kBC))
                continue;
            int fC = triLookup[1][kBC];
            if (fC == t1.second)
            {
                triplets.emplace_back(std::get<0>(kAC), std::get<2>(kAC), std::get<0>(kBC), std::get<2>(kBC), std::get<0>(kAB), std::get<2>(kAB));
                assert(std::get<0>(kAC) == std::get<0>(kAB) && std::get<1>(kAC) == std::get<1>(kBC));
            }
        }
    }
    std::cout << "Total triplets: " <<  triplets.size() << std::endl;
    return triplets.size();
}

int scorePsp(size_t id1, size_t id2, bool writeRes = false)
{
    auto QPB = QQ[0][id1];
    auto QPC = QQ[1][id2];
    auto b1 =  positions[1] - positions[0];
    auto b2 =  positions[2] - positions[0];
    auto t1 =  tt[0][id1];
    auto t2 =  tt[1][id2];
    if (std::abs(t1.angleTo(t2) - b1.angleTo(b2)) * 180.0 / M_PI > 5.0)
        return 0;
    auto Q = fit(t1, t2, b1, b2);
    pss[0] = pss[1] = pss[2] = photostation;
    pss[0].location.shift = positions[0];
    pss[1].location.shift = positions[1];
    pss[2].location.shift = positions[2];
    pss[0].location.rotor = Q;//tions[0];
    pss[1].location.rotor = Q^QPB;//s[2];
    pss[2].location.rotor = Q^QPC;//s[3];
    b1.normalise();
    b2.normalise();
    t1.normalise();
    t2.normalise();
    double realAngle = (positions[1] - positions[0]).angleTo(positions[2] - positions[0]);
    double angle = t1.angleTo(t2);
    if (std::abs(realAngle - angle)*180.0/M_PI > 5.0)
        return 0;

    int inliers = 0;
    for (auto& ms: refined.matchSets)
    {
        int imgA = ms.imgA;
        int imgB = ms.imgB;
        auto p1 = cam_map[imgA];
        auto p2 = cam_map[imgB];
        auto K1 = pss[p1.first].getRawCamera(p1.second).intrinsics.getKMatrix33().inv();
        auto K2 = pss[p2.first].getRawCamera(p2.second).intrinsics.getKMatrix33().inv();
        auto E = pss[p1.first].getRawCamera(p1.second).essentialDecomposition(pss[p2.first].getRawCamera(p2.second));
        auto FM = pss[p1.first].getRawCamera(p1.second).fundamentalTo(pss[p2.first].getRawCamera(p2.second));
        for (auto& m: ms.matches)
        {
            auto kpL = keypoints[p1.first][p1.second][m.featureA];
            auto kpR = keypoints[p2.first][p2.second][m.featureB];

            corecvs::Vector2dd L(kpL.x, kpL.y), R(kpR.x, kpR.y);
            corecvs::Vector3dd L3(L[0], L[1], 1.0), R3(R[0], R[1], 1.0);
            auto lineL = FM.mulBy2dRight(R);
            auto lineR = FM.mulBy2dLeft(L);
            double diff = std::abs(corecvs::Vector3dd(L[0], L[1], 1.0) & lineL);
            double lnorm = std::min(std::sqrt(lineL[0] * lineL[0] + lineL[1] * lineL[1]), std::sqrt(lineR[0] * lineR[0] + lineR[1] * lineR[1]));
            double scaleL, scaleR, foo;
            auto R1 = K2 * R3;
             R1 /= R1[2];
            auto L1 = K1 * L3;
                        L1 /= L1[2];
            E.getScaler(R1, L1, scaleL, scaleR, foo);
            if (scaleL > 0.0 && scaleR > 0.0 && diff / lnorm < pxSimilarityCombine)
            {
                ++inliers;
                if (writeRes)
                {
                    int id = map_fcnt[p1.first][p2.first];
                    bool swap = p1.first > p2.first;
                    fcnt[id][swap ? p2.second * CAM_PER_PS + p1.second : p1.second * CAM_PER_PS + p2.second]++;
                }
            }
        }
    }

 //   std::cout << "INL: " << inliers << std::endl;
    if (writeRes)
    {
        createTriples(id1, id2);
        Mesh3D meshres;
        meshres.switchColor(true);
        corecvs::Vector3dd meanpos(0, 0, 0);
        for (int i = 0; i < 3; ++i)
        {
            meanpos += pss[i].location.shift * (1.0 / 3.0);
        }
        for (int i = 0; i < 3; ++i)
        {
            std::stringstream ss;
            ss << "SP" << ((char)('A' + i));
            pss[i].location.shift -= meanpos;
            pss[i].name = ss.str();

        CalibrationHelpers().drawPly(meshres, pss[i], 50.0);
        }
        meshres.dumpPLY("triples.ply");
    std::cout << (Q*t1).angleTo(b1)*180.0/M_PI << std::endl;
    std::cout << (Q*t2).angleTo(b2)*180.0/M_PI << std::endl;

    }
//    return createTriples(id1, id2);
    return inliers;
}

void detectAndMatchAll()
{
    cam_map.clear();
    std::vector<std::string> filenames;
    for (size_t i = 0; i < 2; ++i)
    {
        for (size_t j = 0; j < CAM_PER_PS; ++j)
        {
            cam_map[filenames.size()] = std::make_pair(i, j);
            filenames.push_back(filenamesUndist[i][j]);
        }
    }
    FeatureMatchingPipeline pipeline(filenames);
    pipeline.add(new KeyPointDetectionStage(detector), true);
    pipeline.add(new DescriptorExtractionStage(descriptor), true);
    pipeline.add(new MatchingPlanComputationStage(), true);
    pipeline.add(new MatchAndRefineStage(descriptor, matcher, inlierB2B), true);
    pipeline.run();

    int id = 0;
    for (int i = 0; i < 2; ++i)
    {
        for (size_t j = 0; j < CAM_PER_PS; ++j)
        {
            keypoints[i][j] = pipeline.images[id++].keyPoints.keyPoints;
        }
    }
    refined = pipeline.refinedMatches;
}
int scorePsp()
{
    int maxInliers = 0;
    int sum = 0;
    int cnt = 0;
    int imax, jmax;
    for (int i = 0; i < CAM_PER_PS * CAM_PER_PS; ++i)
        for (int j = 0; j < CAM_PER_PS * CAM_PER_PS; ++j)
        {
            int sc = scorePsp(i, j);
            if (sc > maxInliers)
            {
                maxInliers = sc;
                imax = i;
                jmax = j;
            }
            sum += sc;
            cnt++;
        }
    std::cout << "maxInliers: " << maxInliers << " from " << imax / CAM_PER_PS << "->" << imax % CAM_PER_PS << ", " << jmax / CAM_PER_PS << "->" << jmax % CAM_PER_PS << " avg: " << sum / cnt << std::endl;
    scorePsp(imax, jmax, true);
    for (int i = 0; i < 3; ++i)
    {
        for (int ii = 0; ii < 3; ++ii)
        {
            int idx = map_fcnt[i][ii];
        std::cout << ((char)('A' + i)) << "->" << ((char)('A' + ii)) << std::endl;
        for (int j = 0; j < CAM_PER_PS; ++j)
        {
            for (int k = 0; k < CAM_PER_PS; ++k)
            {
                std::cout << fcnt[idx][j * CAM_PER_PS + k] << " ";
            }
            std::cout << std::endl;
        }
        }
    }
    return maxInliers;
}

#include "relativeNonCentralRansacSolver.h"
void runNonCentral()
{
    corecvs::RelativeNonCentralRansacSolver::MatchContainer ransac, all;
    for (auto &ms: refined.matchSets)
    {
        auto id1 = cam_map[ms.imgA];
        auto id2 = cam_map[ms.imgB];
        if (id1.first != 0 && id2.first != 0)
            continue;
        if (id2.first != 1 && id1.first != 1)
            continue;
        bool swap = id1.first != 0;
        if (swap)
            std::swap(id1, id2);
        CORE_ASSERT_TRUE_S(id1.first == 0 && id2.first == 1);
        for (auto &m: ms.matches)
        {
            int featureA = swap ? m.featureB : m.featureA;
            int featureB = swap ? m.featureA : m.featureB;
            auto kpA = keypoints[0][id1.second][featureA];
            auto kpB = keypoints[1][id2.second][featureB];
            corecvs::Vector2dd ptA(kpA.x, kpA.y);
            corecvs::Vector2dd ptB(kpB.x, kpB.y);
            if (m.best2ndBest < ransacB2B)
                ransac.emplace_back(id1.second, ptA, id2.second, ptB);
            all.emplace_back(id1.second, ptA, id2.second, ptB);
        }
    }
    photostation.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    photostation.location.shift = corecvs::Vector3dd(0, 0, 0);
    for(int i = 0; i < photostation.cameras.size(); ++i)
        photostation.cameras[i].extrinsics.position /= 1000.0;
    RelativeNonCentralRansacSolver solver(photostation, photostation, ransac, all);
    solver.run();
    auto best = solver.getBestHypothesis();
    std::cout << best.shift << " " << best.rotor << std::endl;
}

void detectRelative(size_t otherId, size_t camIdRef, size_t camIdOther)
{
    std::mt19937 rng;
    corecvs::Quaternion bestQ;
    corecvs::Vector3dd  bestT;
    size_t maxInliers = 0;

    RefinedMatchSet msr;// = refined.matchSets[0];
    bool found = false;
    for (auto& ms: refined.matchSets)
    {
        auto idA = cam_map[ms.imgA];
        auto idB = cam_map[ms.imgB];
        if (idB.first == 0)
            std::swap(idA, idB);
        if (idA.first != 0)
            continue;
        if (idB.first != otherId + 1)
            continue;
        if (idB.second != camIdOther)
            continue;
        if (idA.second != camIdRef)
            continue;
        msr = ms;
        found = true;
        break;
    }
    assert(found);
    auto idA = cam_map[msr.imgA];
    int idRef = msr.imgA;
    int idOther = msr.imgB;
    bool swapem = false;
    if (idA.first != 0)
    {
        swapem = true;
        std::swap(idRef, idOther);
    }
    

    std::vector<std::pair<size_t, size_t>> ransacableMatches, allMatches;
    for (size_t i = 0; i < msr.matches.size(); ++i)
    {
        auto p = swapem ? std::make_pair<size_t, size_t>(msr.matches[i].featureB, msr.matches[i].featureA) : std::make_pair<size_t, size_t>(msr.matches[i].featureA, msr.matches[i].featureB);
        if (msr.matches[i].best2ndBest < ransacB2B)
            ransacableMatches.push_back(p);
        allMatches.push_back(p);
    }

    corecvs::Matrix33 K1 = photostation.cameras[camIdRef  ].intrinsics.getKMatrix33().inv();
    corecvs::Matrix33 K2 = photostation.cameras[camIdOther].intrinsics.getKMatrix33().inv();
    
    corecvs::EssentialDecomposition decBest;
    corecvs::Matrix33 essentialBest;
    std::vector<corecvs::Correspondence> inliersBest;
    int inlierBest = 0;
    for (size_t ir = 0; ir < ransacIterationLimit; ++ir)
    {
        const size_t NSEL = 5;
        std::vector<size_t> selected(NSEL);
        for (size_t j = 0; j < NSEL;)
        {
            selected[j] = rng() % ransacableMatches.size();
            bool isOk = true;
            for (size_t k = 0; k < j; ++k)
                if (selected[k] == selected[j])
                {
                    isOk = false;
                    break;
                }
            if (isOk)
                ++j;
        }
        std::vector<corecvs::Correspondence> cv;
        cv.reserve(NSEL);
        for (auto& id: selected)
        {
            corecvs::Correspondence corr;
            corr.start = K1 * corecvs::Vector2dd(
                    keypoints[0][camIdRef][ransacableMatches[id].first].x,
                    keypoints[0][camIdRef][ransacableMatches[id].first].y);
            corr.end   = K2 * corecvs::Vector2dd(
                    keypoints[otherId + 1][camIdOther][ransacableMatches[id].second].x,
                    keypoints[otherId + 1][camIdOther][ransacableMatches[id].second].y);
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
            std::vector<corecvs::Correspondence> inliers[4];
            for (auto &m: allMatches)
            {
                auto kpRef = keypoints[0][camIdRef][m.first];
                auto kpQue = keypoints[otherId + 1][camIdOther][m.second];
                corecvs::Vector2dd L(kpRef.x, kpRef.y);
                corecvs::Vector2dd R(kpQue.x, kpQue.y);
                auto lineL = FM.mulBy2dRight(R);
                auto lineR = FM.mulBy2dLeft(L);
                double diff = std::abs(corecvs::Vector3dd(L[0], L[1], 1.0) & lineL);
                double lnorm = std::min(std::sqrt(lineL[0] * lineL[0] + lineL[1] * lineL[1]), std::sqrt(lineR[0] * lineR[0] + lineR[1] * lineR[1]));

                if (diff / lnorm < pxSimilarityRansac)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        double scaleL, scaleR, foo;
                        auto R1 = K2 * R;
//                        R1 /= R1[2];
                        auto L1 = K1 * L;
//                        L1 /= L1[2];
                        decompositions[i].getScaler(R1, L1, scaleL, scaleR, foo);
                        if (scaleL > 0.0 && scaleR > 0.0)
                        {
                            ++inlierCnt[i];
                            corecvs::Correspondence corr;
                            corr.start = L1;
                            corr.end = R1;
                            inliers[i].push_back(corr);
                        }
                    }
                }
            }
            for (int i = 0; i < 4; ++i)
            {
                if (inlierCnt[i] > inlierBest)
                {
                    inlierBest = inlierCnt[i];
                    decBest = decompositions[i];
                    inliersBest = inliers[i];
                    essentialBest = F;
                    std::cout << "P: " << otherId << " " << camIdRef << " " << camIdOther << " " << inlierBest << " (" << ((double)inlierBest) / ((double)allMatches.size()) << ")" << std::endl;
                }
            }
        }

    }

    std::vector<corecvs::Correspondence*> corrs;
    corrs.reserve(inliersBest.size());
    for (auto& cc: inliersBest)
        corrs.push_back(&cc);
    auto EM = corecvs::EssentialEstimator().getEssentialLM(corrs, corecvs::Quaternion::FromMatrix(decBest.rotation), decBest.direction);
    EssentialDecomposition decompositions[4];
    EM.decompose(decompositions);

    int closest = 0;
    double minDiff = !(decBest.direction - decompositions[0].direction) + (decBest.rotation.transposed() * decompositions[0].rotation - corecvs::Matrix33(1,0,0,0,1,0,0,0,1)).frobeniusNorm();
    for (int i = 0; i < 4; ++i)
    {
        double diff = !(decBest.direction - decompositions[i].direction) + (decBest.rotation.transposed() * decompositions[i].rotation - corecvs::Matrix33(1,0,0,0,1,0,0,0,1)).frobeniusNorm();
        if (diff < minDiff)
        {
            minDiff = i;
            closest = i;
        }
    }
    decBest = decompositions[closest];
    int before = corrs.size();
    int after = 0;
    auto FM = K1.transposed() * EM * K2;
            for (auto &m: allMatches)
            {
                auto kpRef = keypoints[0][camIdRef][m.first];
                auto kpQue = keypoints[otherId + 1][camIdOther][m.second];
                corecvs::Vector2dd L(kpRef.x, kpRef.y);
                corecvs::Vector2dd R(kpQue.x, kpQue.y);
                auto lineL = FM.mulBy2dRight(R);
                auto lineR = FM.mulBy2dLeft(L);
                double diff = std::abs(corecvs::Vector3dd(L[0], L[1], 1.0) & lineL);
                double lnorm = std::min(std::sqrt(lineL[0] * lineL[0] + lineL[1] * lineL[1]), std::sqrt(lineR[0] * lineR[0] + lineR[1] * lineR[1]));

                if (diff / lnorm < pxSimilarityRansac)
                {
                        double scaleL, scaleR, foo;
                        auto R1 = K2 * R;
//                        R1 /= R1[2];
                        auto L1 = K1 * L;
//                        L1 /= L1[2];
                        decBest.getScaler(R1, L1, scaleL, scaleR, foo);
                        if (scaleL > 0.0 && scaleR > 0.0)
                        {
                            after++;
                        }
                    }
                }
    std::cout << "Should LM? " << before << " -> " << after << std::endl;

    std::cout << otherId << " " << camIdRef << " " << camIdOther << " " << inlierBest << " (" << ((double)inlierBest) / allMatches.size() << ")" << std::endl;
    corecvs::Quaternion QPQ = QQ[otherId][camIdRef * CAM_PER_PS + camIdOther] = photostation.cameras[camIdRef].extrinsics.orientation.conjugated() ^ Quaternion::FromMatrix(decBest.rotation).conjugated() ^ photostation.cameras[camIdOther].extrinsics.orientation;
    corecvs::Quaternion QCQ = photostation.cameras[camIdOther].extrinsics.orientation;
    cnt[otherId][camIdRef * CAM_PER_PS + camIdOther] = inlierBest;
    tt[otherId][camIdRef * CAM_PER_PS + camIdOther] = -((QPQ ^ QCQ.conjugated()) * decBest.direction);
}

std::vector<std::tuple<int, int, int>> works;

struct ParallelRelativeDetector
{
    void operator() (const corecvs::BlockedRange<int> &r) const
    {
        for (int i = r.begin(); i < r.end(); ++i)
        {
            auto t = works[i];
            detectRelative(std::get<0>(t), std::get<1>(t), std::get<2>(t));
        }
    }
};

void detectAllRelative()
{
//    tbb::task_group g;
    for (size_t other = 0; other < 2; ++other)
    {
        for (size_t camRef = 0; camRef < CAM_PER_PS; ++camRef)
        {
            for (size_t camIdOther = 0; camIdOther < CAM_PER_PS; ++camIdOther)
            {
                works.emplace_back(other, camRef, camIdOther);
//                g.run([=]() { detectRelative(other, camRef, camIdOther); });
            }
        }
    }
    corecvs::parallelable_for(0, (int)works.size(), ParallelRelativeDetector());
//    g.wait();
}

void setupData()
{
    std::string prefix = "/hdd_4t/data/roof_v1/roof_1_SP";
    std::string postfix= "_0deg_undist.jpg";
    char pss[] = {'A', 'E'};
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < CAM_PER_PS; ++j)
        {
            std::stringstream ss;
            ss << prefix << pss[i] << j << postfix;
            filenamesUndist[i][j] = ss.str();
        }
    }
    JSONGetter getter("calibration.json");
    CalibrationJob job;
    getter.visit(job, "job");
    photostation = job.photostation;
    for (int i = 0; i < 6; ++i)
    {
        std::cout << photostation.cameras[i].nameId << " " << photostation.cameras[i].extrinsics.position << " ";
        photostation.cameras[i].extrinsics.orientation.printAxisAndAngle();
    }
    std::cout << std::endl;
}

int main()
{
   init_opencv_detectors_provider();
    init_opencv_matchers_provider();
    init_opencv_reader_provider();
    init_opencv_descriptors_provider();
    init_opencv_reader_provider();

    double cntS = 0.0, cntSS = 0.0;
    int NN = 1;
        setupData();
        std::cout << "MATCHING..." << std::endl;
        detectAndMatchAll();
        std::cout << "Running NC" << std::endl;
        runNonCentral();
#if 0
        std::cout << "DETECTING orientations..." << std::endl;
        detectAllRelative();
    
        for (int i = 0; i < 2; ++i)
        {
            std::cout << "SPA->SP" << (char)('B' + i) << std::endl;
            for (int j = 0; j < CAM_PER_PS; ++j)
            {
                for (int k = 0; k < CAM_PER_PS; ++k)
                {
                    QQ[i][j * CAM_PER_PS + k].printAxisAndAngle();
                    std::cout << "(" << cnt[i][j * CAM_PER_PS + k] << ")";
                    std::cout << " " << tt[i][j * CAM_PER_PS + k].normalised() << std::endl;
                    std::cout << "\t\t";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl << std::endl;
        }
        computeAngles();
        double res = scorePsp();
        cntS += res;
        cntSS += res * res;
    }
    cntS /= NN;
    cntSS /= NN;
    cntSS = std::sqrt(cntSS - cntS * cntS);
    std::cout << "Total: " << cntS << "avg (+-" << cntSS * 3.0 << ")" << std::endl;
    pss[0].location.rotor.printAxisAndAngle();
    return 0;
#endif
}

#if 0




double angleee[36][36];
char prefixes[36][2];
corecvs::Quaternion QQ[36][36];


struct FOO
{
    void operator() (const corecvs::BlockedRange<int> &boo) const
    {
        for (int rbrbr= boo.begin(); rbrbr < boo.end(); ++rbrbr)
        {
            std::string base1= "/hdd_4t/data/roof_v1/roof_1_SPA";
            std::string base2= "/hdd_4t/data/roof_v1/roof_1_SPB";
            char cc = prefixes[rbrbr][0];
            char cd = prefixes[rbrbr][1];
            *base1.rbegin() = cd;
            *base2.rbegin() = cc;
            for (int kkk = 0; kkk < 36; ++kkk)
            {
                std::stringstream ss1;
                ss1 << base1 << (kkk / 6) << "_0deg_undist.jpg";
                std::string s1 = ss1.str();
                ss1 =std::stringstream();
                ss1 << base2 << (kkk % 6) << "_0deg_undist.jpg";
                std::string s2 = ss1.str();

                std::string detector = "SURF", descriptor = "SURF", matcher = "BF";
                std::vector<std::string> filenames = {s1, s2}; //"/hdd_4t/data/roof_v1/roof_1_SPH5_0deg.jpg", "/hdd_4t/data/roof_v1/roof_1_SPC5_0deg.jpg"};
                CalibrationJob job;
                JSONGetter getter("proto2_calib.json");
                getter.visit(job, "job");


                std::string camId1 = std::to_string(kkk / 6), camId2 =std::to_string(kkk % 6);
                int id1 = -1, id2 = -1;
                for (int i = 0; i < job.photostation.cameras.size(); ++i)
                {
                    std::cout << job.photostation.cameras[i].nameId << std::endl;
                    if (job.photostation.cameras[i].nameId == camId1)
                        id1 = i;
                    if (job.photostation.cameras[i].nameId == camId2)
                        id2 = i;
                }
                std::cout << "CAM" << camId1 << job.photostation.cameras[id1].intrinsics.getHFov() * 180.0 / M_PI << std::endl;
                corecvs::Matrix33 K1 = job.photostation.cameras[id1].intrinsics.getKMatrix33().inv();
                corecvs::Matrix33 K2 = job.photostation.cameras[id2].intrinsics.getKMatrix33().inv();

                FeatureMatchingPipeline pipeline(filenames);
                pipeline.add(new KeyPointDetectionStage(detector), true);
                pipeline.add(new DescriptorExtractionStage(descriptor), true);
                pipeline.add(new MatchingPlanComputationStage(), true);
                double ransacB2B = 0.8;
                double inlierB2B = 0.9;
                pipeline.add(new MatchAndRefineStage(descriptor, matcher, inlierB2B), true);
                pipeline.run();
#if 0
                std::ofstream ml;
                ml.open("plot_data.m", std::ios_base::out);
                char postfix[] = {'A', 'B'};
                ml << "nameA = '"  << filenames[0] << "';" << std::endl;
                ml << "nameB = '"  << filenames[1] << "';" << std::endl;
                int cnt = 0;
                for (auto& imgd: pipeline.images)
                {
                    ml << "img" << postfix[cnt++] << " = [";
                    for (auto& kp: imgd.keyPoints.keyPoints)
                        ml << kp.x << ", " << kp.y << "; ";
                    ml << "];" << std::endl;
                }
#endif
                std::mt19937 rng;
                corecvs::EssentialMatrix best;
                corecvs::EssentialDecomposition decBest;
                double bestInlier = 0.0;
                int N = 100000;
                double M = 0.001;
                std::vector<int> bestsel;
                std::vector<int> b2b;

                for (auto& ms: pipeline.refinedMatches.matchSets)
                {

                    for (int i = 0; i < ms.matches.size(); ++i)
                        if (ms.matches[i].best2ndBest < ransacB2B)
                            b2b.push_back(i);
#if 0
                    std::cout << "B2B: " << b2b.size() << std::endl;
                    cnt = 0;
                    ml << "matches = [";
                    for (auto& m: ms.matches)
                        ml << m.featureA << ", " << m.featureB << "; ";
                    ml << "]; " << std::endl;
                    ml << "mA" <<  " = [";
                    for (auto& m: ms.matches)
                    {
                        auto kp = pipeline.images[0].keyPoints.keyPoints[m.featureA];
                        ml << kp.x << ", " << kp.y << ";";
                    }
                    ml << "];" << std::endl;
                    ml << "mB" <<  " = [";
                    for (auto& m: ms.matches)
                    {
                        auto kp = pipeline.images[1].keyPoints.keyPoints[m.featureB];
                        ml << kp.x << ", " << kp.y << ";";
                    }
                    ml << "];" << std::endl;
#endif
                   std::vector<int> selected(5);
                    for (int i = 0; i < N; ++i)
                    {
                        int id = 0;
                        while (id < 5)
                        {
                            selected[id] = b2b[rng() % b2b.size()];
                            bool ok = true;
                            if (ms.matches[selected[id]].best2ndBest > ransacB2B)
                                continue;
                            for (int j = 0; j < id; ++j)
                                if (selected[j] == selected[id])
                                {
                                    ok = false;
                                    break;
                                }
                            if (ok) id++;
                        }
                        std::vector<corecvs::Correspondence> cv(5);
                        int ii = 0;
                        for (auto&id : selected)
                        {
                            cv[ii].start =
                                transform(K1,
                                        corecvs::Vector2dd(
                                            pipeline.images[0].keyPoints.keyPoints[ms.matches[id].featureA].x,
                                            pipeline.images[0].keyPoints.keyPoints[ms.matches[id].featureA].y
                                            ));
                            cv[ii].end =
                                transform(K2, 
                                        corecvs::Vector2dd(
                                            pipeline.images[1].keyPoints.keyPoints[ms.matches[id].featureB].x,
                                            pipeline.images[1].keyPoints.keyPoints[ms.matches[id].featureB].y
                                            ));
                            //                std::cout << "Match:" << cv[ii].start << "-" << cv[ii].end << std::endl;
                            ii++;
                        }
                        std::vector<Correspondence*> cl;
                        for (auto& cc: cv)
                            cl.push_back(&cc);
                        auto Fv = corecvs::EssentialEstimator().getEssential5point(cl);

                        for (auto&F : Fv)
                        {
                            int idx = 0;
                            EssentialDecomposition decompositin[4];
                            F.decompose(decompositin);
                            double s1, s2, foo;
                            for (int ijk = 0; ijk < 4; ++ijk)
                            {
                                auto decomposition = decompositin[ijk];
                                int inliercnt = 0;
                                for (int jjjj = 0; jjjj < 5; ++jjjj)
                                {

                                    decomposition.getScaler(corecvs::Vector2dd(cv[jjjj].end[0], cv[jjjj].end[1]), corecvs::Vector2dd(cv[jjjj].start[0], cv[jjjj].start[1]), s1, s2, foo);
                                    if (s1 > 0 && s2 > 0)
                                        inliercnt++;

                                }
                                if (inliercnt < 5) continue;
                                double incnt = .0;
                                for (auto &m : ms.matches)
                                {
                                    corecvs::Vector3dd L(
                                            pipeline.images[0].keyPoints.keyPoints[m.featureA].x,
                                            pipeline.images[0].keyPoints.keyPoints[m.featureA].y, 1.0);
                                    corecvs::Vector3dd R(
                                            pipeline.images[1].keyPoints.keyPoints[m.featureB].x, 
                                            pipeline.images[1].keyPoints.keyPoints[m.featureB].y, 1.0);
                                    L = K1 * L;
                                    R = K2 * R;
                                    L /= L[2];
                                    R /= R[2];
                                    auto line = F * R;
                                    auto line2 =L * F;
                                    double diff = L & line;
                                    double diff2= R & line2;
                                    double lineNorm = std::sqrt(line[0] * line[0] + line[1] * line[1]);
                                    double lineNorm2= std::sqrt(line2[0] * line2[0] + line2[1] * line2[1]);
                                    double scaleL, scaleR, foo;

                                    if (std::abs(diff) / std::min(lineNorm, lineNorm2) < M)
                                    {
                                        //                                        std::cout << std::abs(diff) / std::min(lineNorm, lineNorm2) << std::endl;
                                        decomposition.getScaler(R, L, scaleL, scaleR, foo);
                                        if (scaleL > 0 && scaleR > 0)
                                            incnt++;
                                    }

                                }
                                if (incnt / ms.matches.size() > bestInlier)
                                {
                                    bestInlier = incnt / ms.matches.size();
                                    std::cout << i << ": " << bestInlier << std::endl;
                                    best = F;
                                    bestsel = selected;
                                    decBest = decomposition;
                                }
                            }

                        }

                    }
                }
#if 0
                ml << "realM = [";
                for (auto &m : pipeline.refinedMatches.matchSets[0].matches)
                {
                    corecvs::Vector3dd L(
                            pipeline.images[0].keyPoints.keyPoints[m.featureA].x,
                            pipeline.images[0].keyPoints.keyPoints[m.featureA].y, 1.0);
                    corecvs::Vector3dd R(
                            pipeline.images[1].keyPoints.keyPoints[m.featureB].x, 
                            pipeline.images[1].keyPoints.keyPoints[m.featureB].y, 1.0);
                    L = K1 * L;
                    R = K2 * R;
                    L /= L[2];
                    R /= R[2];
                    auto line = best * R;
                    double diff = L & line;
                    double lineNorm = std::sqrt(line[0] * line[0] + line[1] * line[1]);
                    auto line2 =L * best;
                    double lineNorm2= std::sqrt(line2[0] * line2[0] + line2[1] * line2[1]);
                    double scaleL, scaleR, foo;

                    if (std::abs(diff) / std::min(lineNorm, lineNorm2) < M)
                    {
                        decBest.getScaler(corecvs::Vector2dd(R[0], R[1]), corecvs::Vector2dd(L[0], L[1]), scaleL, scaleR, foo);
                        if (scaleL > 0 && scaleR > 0)
                        {
                            corecvs::Vector3dd L(
                                    pipeline.images[0].keyPoints.keyPoints[m.featureA].x,
                                    pipeline.images[0].keyPoints.keyPoints[m.featureA].y, 1.0);
                            corecvs::Vector3dd R(
                                    pipeline.images[1].keyPoints.keyPoints[m.featureB].x, 
                                    pipeline.images[1].keyPoints.keyPoints[m.featureB].y, 1.0);
                            ml << L[0] << ", " << L[1] << ", " << R[0] << ", " << R[1] << "; ";
                        }
                    }

                }
                ml << "];" << std::endl;
                ml << "selM = [";
                for (auto &id : bestsel)
                {
                    auto& m = pipeline.refinedMatches.matchSets[0].matches[id];
                    corecvs::Vector3dd L(
                            pipeline.images[0].keyPoints.keyPoints[m.featureA].x,
                            pipeline.images[0].keyPoints.keyPoints[m.featureA].y, 1.0);
                    corecvs::Vector3dd R(
                            pipeline.images[1].keyPoints.keyPoints[m.featureB].x, 
                            pipeline.images[1].keyPoints.keyPoints[m.featureB].y, 1.0);
                    L = K1 * L;
                    R = K2 * R;
                    auto line = best * R;
                    auto line2 =L * best;
                    double diff = L & line;
                    double lineNorm = std::sqrt(line[0] * line[0] + line[1] * line[1]);
                    double lineNorm2= std::sqrt(line2[0] * line2[0] + line2[1] * line2[1]);

                    if (std::abs(diff) / lineNorm < M)
                    {
                        corecvs::Vector3dd L(
                                pipeline.images[0].keyPoints.keyPoints[m.featureA].x,
                                pipeline.images[0].keyPoints.keyPoints[m.featureA].y, 1.0);
                        corecvs::Vector3dd R(
                                pipeline.images[1].keyPoints.keyPoints[m.featureB].x, 
                                pipeline.images[1].keyPoints.keyPoints[m.featureB].y, 1.0);
                        ml << L[0] << ", " << L[1] << ", " << R[0] << ", " << R[1] << "; ";
                    }

                }
                ml << "];" << std::endl;
#endif
                std::cout << "Best inlier: " << bestInlier << std::endl << "F: " << best << std::endl;    std::cout << "R: " << corecvs::Quaternion::FromMatrix(decBest.rotation) << " S: " << decBest.direction << std::endl;
                auto Q = corecvs::Quaternion::FromMatrix(decBest.rotation);
                angleee[rbrbr][kkk] = std::acos(Q[3]) * 2.0 / M_PI * 180.0;
                QQ[rbrbr][kkk] = job.photostation.cameras[id2].extrinsics.orientation.conjugated() ^ job.photostation.cameras[id1].extrinsics.orientation ^ Q;
        }
        std::cout << "PREFIX: "  << prefixes[rbrbr][0] << " > " << prefixes[rbrbr][1] << " : " << std::endl << "PREFIX:";
        for (int jjj = 0; jjj < 36; ++jjj)
        {
            std::cout << angleee[rbrbr][jjj] << ", ";
            if ((jjj + 1) % 6 == 0)
                std::cout << std::endl << "PREFIX";
        }
        std::cout << std::endl;
    }
}
};


int main(int, char**) {
    init_opencv_detectors_provider();
    init_opencv_matchers_provider();
    init_opencv_reader_provider();
    init_opencv_descriptors_provider();
    init_opencv_reader_provider();
    int idx = 0;

    char to = 'I';
    for (char cc='A'; cc <= to; ++cc)
    {
        for (char cd='A'; cd < cc; ++cd)
        {
            prefixes[idx][0] = cc;
            prefixes[idx++][1] = cd;
        }
    }
    corecvs::parallelable_for(0, 36, FOO());
    idx = 0;
    double QINLIER = M_PI / 24;
    for (char cc = 'A'; cc <= to; ++cc)
    {
        for (char cd = 'A'; cd < cc; ++cd)
        {
            std::cout << cc << " " << cd << " ";
            corecvs::Quaternion best = QQ[idx][0];
            int bestCnt = 0;
            for (int kkk = 0; kkk < 36; ++kkk)
            {
                int ccnt = 0;
                corecvs::Quaternion Q = QQ[idx][kkk];
                for (int lll = 0; lll < 36; ++lll)
                {
                    corecvs::Quaternion Q2 = QQ[idx][lll];
                    double diff = std::acos((Q2.conjugated() ^ Q)[3]);
                    if (diff * 2.0 > M_PI) diff = M_PI - diff;
                    diff = std::abs(diff);
                    if (diff < QINLIER)
                        ccnt++;
                }
                if (ccnt > bestCnt)
                {
                    bestCnt = ccnt;
                    best = Q;
                }
            }
            best.printAxisAndAngle();
            std::cout << " (" << bestCnt << " inlying)" << std::endl;
            idx++;
            std::cout << std::endl;
        }
    }
    idx = 0;
    for (char cc = 'A'; cc <= to; ++cc)
    {
        for (char cd = 'A'; cd < cc; ++cd)
        {
            std::cout << cc << " " << cd << " ";
            for (int kkk = 0; kkk < 36; ++kkk)
            {
                std::cout << angleee[idx][kkk] << ", ";
                if ((kkk + 1) % 6 == 0)
                    std::cout << std::endl;
            }
            idx++;
            std::cout << std::endl;
        }
    }

}
#endif
#endif
