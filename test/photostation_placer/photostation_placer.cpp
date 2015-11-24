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

#include "openCvFileReader.h"
#include "featureMatchingPipeline.h"
#include "openCvDescriptorExtractorWrapper.h"
#include "openCvFeatureDetectorWrapper.h"
#include "openCvDescriptorMatcherWrapper.h"
#include "multicameraTriangulator.h"
#include "undirectedGraph.h"
#include "multiPhotostationScene.h"
#include "essentialEstimator.h"

corecvs::Vector2dd transform(corecvs::Matrix33 iK, corecvs::Vector2dd uv)
{
    corecvs::Vector3dd uv1 = corecvs::Vector3dd(uv[0], uv[1], 1.0);
    auto vv = iK * uv1;
    vv /= vv[2];
    return corecvs::Vector2dd(vv[0], vv[1]);
}


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
