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
#include <set>
#include <chrono>
#include <random>
#include <ctime>

#include "calibrationHelpers.h"
#include "calibrationJob.h"
#include "calibrationLocation.h"
#include "mesh3d.h"
#include "jsonSetter.h"
#include "jsonGetter.h"
#include "log.h"
#include "pnpSolver.h"

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


corecvs::Vector3dd convertVector(const corecvs::Vector3dd& geodesic)
{
    auto vec = geodesic;
    std::swap(vec[0], vec[1]);
    return vec;
}

// For easy control over ps number
const int PSN = 9;
const int CP = 6;
corecvs::Vector3dd mp(0, 0, 0);
//#if 1

// Obsolete
std::vector<PointObservation__> parsePois(CalibrationJob &calibration, const std::string &filename, int camIdOffset = 10, bool distorted = true, bool less10Cams = true)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios_base::in);

    std::vector<PointObservation__> res;

    int N;
    ifs >> N;
    for (int i = 0; i < N; ++i)
    {
        std::string label;
        corecvs::Vector3dd pos;
        int M;
        std::string lstr;
        while (!lstr.size() && (bool)ifs)
        {
            std::getline(ifs, lstr);
            if (lstr.size() && *lstr.rbegin() == '\r')
                lstr.resize(lstr.size() - 1);
        }
        assert(lstr.size());
        std::stringstream ss(lstr);
        std::cout << "IN: " << lstr << std::endl;

        ss >> label;
        if (label.size() < lstr.size())
        {
            ss >> pos[0] >> pos[1] >> pos[2];
        }
        ifs >> M;

        std::vector<PointProjection> projections;
        PointObservation__ observation;
        observation.worldPoint = convertVector(pos);
        observation.updateable = label.size() == lstr.size();
        observation.label = label;

        std::cout << "POI: LABEL: " << label << ": ";
        if (!observation.updateable) std::cout << observation.worldPoint;
        std::cout << std::endl;

        for (int j = 0; j < M; ++j)
        {
            std::string filename;
            double x, y;

            ifs >> filename >> x >> y;
            if (x < 0 || y < 0) continue;
			std::cout << filename << std::endl;

            int psId = filename[camIdOffset - 1] - 'A';

            int camId = less10Cams ?
                (filename[camIdOffset] - '0')
                :((filename[camIdOffset] - '0') * 10 + filename[camIdOffset + 1] - '0') - 1;
            PointProjection proj;
            proj.projection = corecvs::Vector2dd(x, y);
            if (distorted)
            {
                proj.projection = calibration.photostation.cameras[camId].distortion.mapBackward(proj.projection);
            }
            proj.cameraId = camId;
            proj.photostationId = psId;
            if (psId < PSN)
                projections.push_back(proj);
            std::cout << "POI: LABEL: " << label << ": CAM: " << filename << " (" << camId << "|" << psId << ")" << proj.projection << std::endl;

        }
        observation.projections = projections;
        res.push_back(observation);
    }

    return res;

}

void storePois(const std::vector<PointObservation__> &observations, const std::string &filename, const std::string &prefix, CalibrationJob &calibration)
{
    std::ofstream ofs;
    ofs.open(filename, std::ios_base::out);

    ofs << observations.size();
    for (auto& o: observations)
    {
        ofs << o.label << " " << o.worldPoint[0] << " " << o.worldPoint[1] << " " << o.worldPoint[2] << std::endl;
        ofs << o.projections.size() << std::endl;
        for (auto& p: o.projections)
        {
            auto projection = calibration.photostation.cameras[p.cameraId].distortion.mapForward(p.projection);
            ofs << "roof_v1_SP" << ((char)('A' + p.photostationId)) << p.cameraId << "_0deg.jpg" << " " << projection[0] << " " << projection[1] << std::endl;
        }
    }
}

struct exp_desc
{
    std::vector<std::string> allowedPs;
    PhotostationPlacerOptimizationType type;
    bool separate_cameras;
    std::string title;
};

exp_desc experiments[] = 
{
    {
        std::vector<std::string>({"SPA", "SPB", "SPC", "SPD", "SPE", "SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS,
      false,
      "f,cx,cy,common,all"
    },
    {
        std::vector<std::string>({"SPA", "SPB", "SPC", "SPD", "SPE", "SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS,
      false,
      "f,common,all"
    },
    {
        std::vector<std::string>({"SPA", "SPB", "SPC", "SPD", "SPE", "SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS,
      false,
      "common,all"
    },
    {
        std::vector<std::string>({"SPA", "SPB", "SPC", "SPD", "SPE", "SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS,
      true,
      "f,cx,cy,separate,all"
    },
    {
        std::vector<std::string>({"SPA", "SPB", "SPC", "SPD", "SPE", "SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS,
      false,
      "f,cx,cy,common,reduced"
    },
    {
        std::vector<std::string>({"SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::NON_DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::TUNE_GPS,
      false,
      "common,reduced,trans"
    },
    {
        std::vector<std::string>({"SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS  ,
      false,
      "common,reduced"
    },
    {
        std::vector<std::string>({"SPA", "SPB", "SPC", "SPD", "SPE", "SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::NON_DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::TUNE_GPS,
      false,
      "01.02.2016,common,all,trans"
    },
    {
        std::vector<std::string>({"SPA", "SPB", "SPC", "SPD", "SPE", "SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::NON_DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::TUNE_GPS | PhotostationPlacerOptimizationType::FOCALS,
      false,
      "01.02.2016,f,common,all,trans"
    },
    {
        std::vector<std::string>({"SPA", "SPB", "SPC", "SPD", "SPE", "SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::NON_DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::DEGENERATE_TRANSLATIONS | PhotostationPlacerOptimizationType::TUNE_GPS | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS,
      false,
      "01.02.2016f,principal,common,all,trans"
    },
    {
        std::vector<std::string>({"SPA", "SPB", "SPC", "SPD", "SPE", "SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS ,
      false,
      "01.02.2016,common,all"
    },
    {
        std::vector<std::string>({"SPA", "SPB", "SPC", "SPD", "SPE", "SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS,
      false,
      "01.02.2016,f,common,all"
    },
    {
        std::vector<std::string>({"SPA", "SPB", "SPC", "SPD", "SPE", "SPF", "SPG", "SPH", "SPI"}),
      PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS  | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS,
      false,
      "01.02.2016f,principal,common,all"
    },
};


struct TransformEstimationFunctor : public FunctionArgs
{
	std::vector<SceneFeaturePoint*> refPois;
	TransformEstimationFunctor(std::vector<SceneFeaturePoint*> &refPois, int projCnt) : FunctionArgs(7, projCnt*2), refPois(refPois)
	{
	}
	void operator() (const double *in, double *out)
	{
		int argout = 0;
		corecvs::Affine3DQ foo;
		for (int i = 0; i < 3; ++i)
			foo.shift[i] = in[i];
		for (int j = 0; j < 4; ++j)
			foo.rotor[j] = in[3 + j];
		foo.rotor.normalise();
		for (auto spt: refPois)
		{
			for (auto &op: spt->observations__)
			{
				auto& o = op.second;
				auto cam = o.camera;
				auto ps  = o.cameraFixture;
				auto pt  = o.observation;
				auto ptw = spt->position;
				
				ptw = foo.applyInv(ptw);
				auto err = (ps->reprojectionError(ptw, pt, cam));
				out[argout++] = err[0];
				out[argout++] = err[1];
			}
		}
		CORE_ASSERT_TRUE_S(argout == outputs);
	}
};


corecvs::Affine3DQ optimizeTransform(std::vector<SceneFeaturePoint*> &pois, corecvs::Affine3DQ &init, int projCnt)
{
	std::vector<double> in = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, out(projCnt*2);
	LevenbergMarquardt lm(1000);
	TransformEstimationFunctor tef(pois, projCnt);
	lm.f = &tef;
	auto res = lm.fit(in, out);
	corecvs::Affine3DQ T;
	for (int i = 0; i < 3; ++i)
		T.shift[i] = res[i];
	for (int i = 0; i < 4; ++i)
		T.rotor[i] = res[3 + i];
	T.rotor.normalise();
	return T;
}

corecvs::Affine3DQ estimateTransform(std::vector<SceneFeaturePoint*> &pois)
{
	int pcnt = 0;
    std::vector<corecvs::Vector3dd> centers, directions, points3d;
    for (auto spt: pois)
    {
        for (auto &op: spt->observations__)
        {
            auto& o = op.second;
            auto cam = o.camera;
            auto ps  = o.cameraFixture;
            auto pt  = o.observation;
            auto ptw = spt->position;

            centers.push_back(ps->getWorldCamera(cam).extrinsics.position);
            directions.push_back(ps->getWorldCamera(cam).rayFromPixel(pt).a.normalised());
            points3d.push_back(ptw);
            pcnt++;
        }
    }

    auto hypothesis = corecvs::PNPSolver::solvePNP(centers, directions, points3d);
    int inliers = 0;
    int bestHypo = 0;
    double bestRI = 0.0;
    for (auto& hypo: hypothesis)
    {
        int curInliers = 0;
        double inlierThreshold = 3.0;
        double sqerr = 0.0;
        for (auto spt: pois)
        {
            for (auto &op: spt->observations__)
            {
                auto&o = op.second;
                auto cam = o.camera;
                auto ps  = o.cameraFixture;
                auto pt  = o.observation;
                auto ptw = spt->position;

                ptw = hypo.applyInv(ptw);

				auto err = (ps->angleError(ptw, pt, cam));
                if (err < inlierThreshold)
				{
                    curInliers++;
                    sqerr += err * err;
				}
            }
        }
        sqerr /= curInliers;
        if (curInliers > inliers || (curInliers == inliers && sqerr < bestRI))
        {
            inliers = curInliers;
            bestHypo = &hypothesis[0] - &hypo;
            bestRI = sqerr;
        }
    }
    std::cout << "Error transformation: " << inliers << " " << pois.size() << " degdiff " << std::sqrt(bestRI) << std::endl;
	std::cout << hypothesis[bestHypo] << std::endl;
	auto T = optimizeTransform(pois, hypothesis[bestHypo], pcnt);
	std::cout << T << std::endl;
    return T;
}

void validate_pois(std::vector<SceneFeaturePoint*> &poiss, bool tune = false, bool tuneOnlyChess = true, double tuneOn = 30.0)
{
    std::vector<SceneFeaturePoint*> pois2;
	std::set<CameraFixture*> fixtures;
    for (auto poi: poiss)
    {
        if (poi->observations__.size() < 2)
            continue;
        double closest = 1e100;
        std::vector<WPP> observations;
        for (auto& obsp: poi->observations__)
        {
            auto& obs = obsp.second;
            double dist = !(poi->position - obs.cameraFixture->location.shift);
            if (dist < closest)
                closest = dist;
            fixtures.insert(obs.cameraFixture);
        }
        if (closest < tuneOn && ((poi->name[0] == 'C' && poi->name[1] == 'H' && poi->name[2] == 'E') || !tuneOnlyChess))
            pois2.push_back(poi);

    }
	corecvs::Vector3dd meanPoint(0.0, 0.0, 0.0);
	for (auto &p: fixtures)
		meanPoint += p->location.shift;
	meanPoint /= fixtures.size();

    auto tes = estimateTransform(pois2);
    std::cout << "Estimated: "  << tes << std::endl;
    std::cout << "POI,views,combinations,mean offset,rmse,distClosest,relClosest,distMean,relMean,group" << std::endl;
	std::string group = tune ? tuneOnlyChess ? "tune_chess" : "tune_all" : "no_tune";
    int ccnt = 0, acnt = 0;
    double chessAbsSSE = 0.0, allAbsSSE = 0.0, allRelSSE = 0.0, chessRelSSE = 0.0;
    for (auto poi: poiss)
    {
        if (poi->observations__.size() < 2)
            continue;
        corecvs::MulticameraTriangulator mct;    
        double closest = 1e100;// mean = (poi->position - meanPoint);
        std::vector<WPP> observations;
        for (auto& obsp: poi->observations__)
        {
            auto& obs = obsp.second;
            mct.addCamera(obs.cameraFixture->getMMatrix(obs.camera), obs.observation);
            double dist = !(poi->position - obs.cameraFixture->location.shift);
            observations.push_back(WPP(obs.cameraFixture, obs.camera));
            if (dist < closest)
                closest = dist;
        }
        auto res = mct.triangulateLM(mct.triangulate());

        int n = (int)observations.size();
        for (int cnt = 2; cnt <= observations.size(); ++cnt)
        {
            std::vector<bool> foo(n);
            std::fill(foo.begin() + n - cnt, foo.end(), true);

            Vector3dd mean(0, 0, 0);
            double sqrs = 0.0;
            double pcnt = 0.0;
            do
            {
                corecvs::MulticameraTriangulator mct2;
                for (int i = 0; i < n; ++i)
                    if (foo[i])
                    {
                        auto& obs = poi->observations__[observations[i]];
                        mct2.addCamera(obs.cameraFixture->getMMatrix(obs.camera), obs.observation);
                    }
                auto res = mct2.triangulateLM(mct2.triangulate());
                pcnt += 1.0;
                if (tune)
                    res = tes.apply(res);
                sqrs += !(res - poi->position) * !(res - poi->position);
                mean += res;
            } while (std::next_permutation(foo.begin(), foo.end()));
            mean /= pcnt;
            sqrs = std::sqrt(sqrs / pcnt);
            if (pcnt == 1 && closest < 30.0)
			{
				double abssse = sqrs * sqrs;
				double md = !(poi->position - meanPoint);
				double relsse = sqrs * sqrs / md / md;
				if (poi->name[0] == 'C' && poi->name[1] == 'H' && poi->name[2] == 'E')
				{
					chessAbsSSE += abssse;
					chessRelSSE += relsse;
					ccnt++;
				}
				allAbsSSE += abssse;
				allRelSSE += relsse;
				acnt++;
			}
            std::cout << poi->name << "," << cnt << "," << pcnt << ",";
            auto v = mean - poi->position;
            for (int iii = 0; iii < 3; ++iii)
                std::cout << v[iii] << ", ";
            std::cout << sqrs << ", " << closest << "," << sqrs / closest * 100.0 << "%," << !(meanPoint - poi->position) <<", " << sqrs / (!(meanPoint - poi->position)) * 100 << "%" << group << std::endl;
        }
    }
	std::cout << "VERDICT: CHESS: " << std::sqrt(chessAbsSSE / ccnt) << "m/" << std::sqrt(chessRelSSE / ccnt) * 100.0 << "% ALL: " << std::sqrt(allAbsSSE/acnt) << "m/" << std::sqrt(allRelSSE / ccnt) * 100.0 << "%" << std::endl;
}

void validate_cams(std::vector<CameraFixture*> fixtures, std::vector<FixtureCamera*> cameras, std::vector<FixtureCamera*> refCameras, ReconstructionFixtureScene *scene)
{
    std::unordered_map<FixtureCamera*, std::vector<CameraFixture*>> mmp;
    for (auto cf: fixtures)
        for (auto cam: cf->cameras)
        {
            mmp[cam].push_back(cf);
        }
    std::unordered_map<FixtureCamera*, int> map;
    for (auto& cam: cameras)
        map[cam] = &cam - &cameras[0];

    std::cout << "Cam,photostations,calib_focal,new_focal,focal_diff,calib_cx,calib_cy,new_cx,new_cy,diff_principal" << std::endl;
    for (auto camp: mmp)
    {
        auto cam = camp.first;
        std::cout << "Cam " << cam->nameId << ",";
        for (auto fix: camp.second)
        {
            if (fix && cam && map[cam] < refCameras.size())
            {
            std::cout << fix->name << " ";
            }
        }
        double focalPrev = refCameras[map[cam]]->intrinsics.focal[0];
        double focalNew = cam->intrinsics.focal[0];
        double relDiff = focalNew / focalPrev - 1.0;
        Vector2dd principalPrev = refCameras[map[cam]]->intrinsics.principal;
        Vector2dd principalNew = cam->intrinsics.principal;
        std::cout <<"," << focalPrev << "," << focalNew << "," << relDiff*100 << "%," << principalPrev << "," << principalNew << "," << (!(principalPrev-principalNew)) << "" << std::endl;
		std::cout << "Ps,gps_pos,new_pos,diff,len" << std::endl;
		for (auto& init: scene->initializationData)
		{
			if (init.second.initializationType != PhotostationInitializationType::GPS)
				continue;
			auto ps = init.first;
			auto sh = init.second.initData.shift;
			auto re = ps->location.shift;
			std::cout << ps->name << ", " << sh << ", " << re << ", " << sh - re << ", " << !(sh - re) << std::endl;
		}
    }
}

void run_exp(exp_desc exp)
{
    std::cout << exp.title << std::endl;
    L_ERROR << "STARTING_EXP";
    bool separate_cameras = exp.separate_cameras;
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

    std::vector<std::string> allowedPs = exp.allowedPs;


    char psPrefixes[9] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};

    std::string topconBase(std::getenv("TOPCON_DIR"));
    std::string prefix = topconBase + "data/tests/reconstruction/GGGGGGGGG/roof_1_SP";
    std::string postfix= "_0deg_undist.jpg";

    std::vector<std::vector<std::string>> images;
    images.resize(PSN);
    for (int i = 0; i < PSN; ++i)
    {
        images[i].resize(CP);
        for (int j = 0; j < CP; ++j)
        {
            std::stringstream ss;
            ss << prefix << psPrefixes[i] << j << postfix;
            images[i][j] = ss.str();
        }
    }

    corecvs::Photostation photostation;
    JSONGetter getter("calibration.json");
    CalibrationJob job;
    getter.visit(job, "job");

    photostation = job.photostation;
    for (size_t i = 0; i < CP; ++i)
    {
        photostation.cameras[i].extrinsics.position /= 1e3;
    }

#if 0
    for (int i = 0; i < PSN; ++i)
    {
        auto& ps = pp.calibratedPhotostations[i] = photostation;
        std::stringstream ss;
        ss << "SP" << ((char)('A' + i));
        ps.name = ss.str();
    }
#endif
    auto pois = parsePois(job, "pois_m15.txt",10, true, true);

    ReconstructionFixtureScene rfs;
    std::vector<FixtureCamera*> cameras;
    std::vector<CameraFixture*> photostations;
    std::vector<SceneFeaturePoint*> poiss;
    std::vector<FixtureCamera*> refCameras;
    if (!separate_cameras)
    {
        for (int i = 0; i < CP; ++i)
        {
            cameras.push_back(rfs.createCamera());
            refCameras.push_back(rfs.createCamera());
            cameras.rbegin()[0]->intrinsics = photostation.cameras[i].intrinsics;
            cameras.rbegin()[0]->extrinsics = photostation.cameras[i].extrinsics;
            cameras.rbegin()[0]->distortion = photostation.cameras[i].distortion;
            cameras.rbegin()[0]->nameId = photostation.cameras[i].nameId;
            *refCameras.rbegin()[0] = **cameras.rbegin();
        }
    }
    else
    {
        for (int j = 0; j < PSN; ++j)
        for (int i = 0; i < CP; ++i)
        {
            refCameras.push_back(rfs.createCamera());
            cameras.push_back(rfs.createCamera());
            cameras.rbegin()[0]->intrinsics = photostation.cameras[i].intrinsics;
            cameras.rbegin()[0]->extrinsics = photostation.cameras[i].extrinsics;
            cameras.rbegin()[0]->distortion = photostation.cameras[i].distortion;
            cameras.rbegin()[0]->nameId = photostation.cameras[i].nameId;
            *refCameras.rbegin()[0] = **cameras.rbegin();
        }
    }
    for (int i = 0; i < PSN; ++i)
    {
        photostations.push_back(rfs.createCameraFixture());
        for (int j = 0; j < CP; ++j)
        {
            int camId = separate_cameras ? i * CP + j : j;
            rfs.addCameraToFixture(cameras[camId], photostations[i]);
            rfs.images[WPP(photostations[i], cameras[camId])] = images[i][j];
        }
        rfs.initializationData[photostations[i]].initializationType = PhotostationInitializationType::GPS;
        rfs.initializationData[photostations[i]].initData.shift = positions[psPrefixes[i] - 'A'];
        std::stringstream ss;
        ss << "SP" << ((char)('A' + i));
        rfs.fixtures[i]->name = ss.str();
    }
    for (auto poi: pois)
    {
        auto sfp = rfs.createFeaturePoint();
        sfp->position = poi.worldPoint;
        sfp->name = poi.label;
        for (auto o: poi.projections)
        {
            //int camId = separate_cameras ? o.cameraId + CP  * o.photostationId : o.cameraId;
            SceneObservation obs;
            obs.camera = cameras[o.cameraId];
            obs.cameraFixture = photostations[o.photostationId];
            obs.observation = o.projection;
            sfp->observations[obs.camera] = obs;
            sfp->observations__[WPP(obs.cameraFixture, obs.camera)] = obs;
        }
        poiss.push_back(sfp);
    }
    rfs.placingQueue = photostations;
    for (auto ptr: photostations)
    {
        std::cout << "Checking if " << ptr->name << " is allowed..." << std::endl;
        if (std::find(allowedPs.begin(), allowedPs.end(), ptr->name) == allowedPs.end())
        {
            std::cout << "Deleting " << ptr->name << std::endl;
           rfs.deleteCameraFixture(ptr, false);
        }
        std::cout << ptr->name << " is Ok" << std::endl;
    }
    std::cout << "Launching psp" << std::endl;

    PhotostationPlacer pp;
    pp.scene = &rfs;
    pp.optimizationParams = exp.type;
//    pp.fullRun();
    
    L_ERROR << "Starting full run" ;
    L_ERROR << "Detecting features";
    pp.detectAll();
    L_ERROR << "Initalizing";
    pp.initialize();
    L_ERROR << "Fitting";
	pp.fit(exp.type, 100);
    L_ERROR << "Appending";

    while(pp.scene->placingQueue.size())
	{
        L_ERROR << "Appending" << (*pp.scene->placingQueue.begin())->name ;
        pp.appendPs();
        L_ERROR << "Fitting";
//      if (scene->is3DAligned)
//  	    fit(PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS, 100);
  //    else
    //   fit(PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS, 200);
        pp.fit(exp.type, 100);

        std::stringstream ss;
        ss << (*pp.scene->placedFixtures.rbegin())->name << "_app.ply";
        pp.dumpMesh(ss.str());
	}
	pp.fit(exp.type, 100000);
    pp.dumpMesh("final.ply");
    JSONSetter setter("psp_finished.json");
//    directory->accept<JSONSetter>(setter);
    setter.visit(*pp.scene, "scene");

    validate_pois(poiss, false, false, 30.0);
    validate_pois(poiss, true, false, 30.0);
    validate_pois(poiss, true, true, 30.0);
    validate_cams(rfs.fixtures, cameras, refCameras, pp.scene);
    L_ERROR << "FINISHING_EXP";
}


int main(int argc, char** argv)
{
    init_opencv_detectors_provider();
    init_opencv_matchers_provider();
    init_opencv_reader_provider();
    init_opencv_descriptors_provider();
    init_opencv_reader_provider();

	if (argc != 2)
	{
		std::cout << "Available settings:" << std::endl;
		for (auto& exp: experiments)
		{
			int id = &exp - experiments;
			std::cout << id << ": " << exp.title << std::endl;
			std::cout << "\tAllowed ps: ";
			for (auto& p: exp.allowedPs)
				std::cout << p << " ";
			std::cout << std::endl;
			std::cout << "\tOptimization options:";
#define IFP(cond, title) \
			if (!!(exp.type & PhotostationPlacerOptimizationType::cond)) \
				std::cout << title << " ";
			IFP(FOCALS, "Focals");
			IFP(PRINCIPALS, "Principals");
			IFP(TUNE_GPS, "Tune_gps");
			IFP(NON_DEGENERATE_ORIENTATIONS, "Non-degenerate_orientations");
			IFP(DEGENERATE_ORIENTATIONS, "Degenerate_orientations");
			IFP(NON_DEGENERATE_TRANSLATIONS, "Non-degenerate_translations");
			IFP(DEGENERATE_TRANSLATIONS, "Degenerate translations");
			IFP(POINTS, "Points");
			std::cout << std::endl;
			std::cout << "\tSeparated cameras: " << (exp.separate_cameras ? "True" : "False") << std::endl;
			std::cout << std::endl;
		}
		return 0;
	}

    int id = std::stoi(std::string(argv[1]));
    std::cout << "Running " << id << std::endl;
    run_exp(experiments[id]);
#if 0
    exp_desc desc;
    desc.allowedPs = std::vector<std::string>({"SPA", "SPB", "SPC"});
    desc.separate_cameras = false;
    run_exp(desc);
#endif


    return 0;
}
//#endif
