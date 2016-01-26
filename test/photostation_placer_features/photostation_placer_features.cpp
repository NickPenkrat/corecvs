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
#if 0

// Obsolete
std::vector<PointObservation__> parsePois(CalibrationJob &calibration, const std::string &filename, int camIdOffset = 3, bool distorted = false, bool less10Cams = true)
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

void setup()
{
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

    for (auto& pos: positions)
        std::cout << pos << " ";
    std::cout << std::endl;

	for (int i = 0; i < 9; ++i)
	{
		mp += positions[i] * (1.0 / 9.0);
	}

    PhotostationPlacer pp;
    char psPrefixes[9] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};

    std::string topconBase(std::getenv("TOPCON_DIR"));
    std::string prefix = topconBase + "data/tests/reconstruction/GGGGGGGGG/roof_1_SP";
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
    pp.psInitData.resize(PSN);
    for (int i = 0; i < PSN; ++i)
    {
        pp.psInitData[i].gpsData = positions[psPrefixes[i] - 'A'];
        pp.psInitData[i].initializationType = PhotostationInitializationType::GPS;
    }

    corecvs::Photostation photostation;
    JSONGetter getter("calibration.json");
    CalibrationJob job;
    getter.visit(job, "job");

    photostation = job.photostation;
    for (size_t i = 0; i < photostation.cameras.size(); ++i)
    {
        photostation.cameras[i].extrinsics.position /= 1e3;
    }

    pp.calibratedPhotostations.resize(PSN);
    for (int i = 0; i < PSN; ++i)
    {
        auto& ps = pp.calibratedPhotostations[i] = photostation;
        std::stringstream ss;
        ss << "SP" << ((char)('A' + i));
        ps.name = ss.str();
    }

    JSONSetter *setter = new JSONSetter("pp.json");
    setter->visit(pp, "reconstruction data");
    delete setter;
}

int main()
{
    init_opencv_detectors_provider();
    init_opencv_matchers_provider();
    init_opencv_reader_provider();
    init_opencv_descriptors_provider();
    init_opencv_reader_provider();

    setup();

    // actually we need only PS
    JSONGetter ggetter("calibration.json");
    CalibrationJob job;
    ggetter.visit(job, "job");
    auto pois = parsePois(job, "pois_m15.txt",10, true, true);
    for (auto &p: pois)
    {
        std::cout << p.label << " " << p.worldPoint << " ";
        for (auto &pp: p.projections)
            std::cout << pp.cameraId << " " << pp.photostationId << " " << pp.projection << " ";
        std::cout << std::endl;
    }

	for (auto &o: pois)
	{
		std::cout << o.label << " " << o.worldPoint << " " << o.worldPoint - mp << " " << !(o.worldPoint - mp) << std::endl;
	}


    PhotostationPlacer pp;
    JSONGetter getter("pp.json");
    getter.visit(pp, "reconstruction data");

    pp.fullRun();

    pp.dumpMesh("reconstruction.res.ply", true);

    auto res = pp.verify(pois);
    corecvs::Vector3dd meanpos(0, 0, 0);
    for (int i = 0; i < PSN; ++i)
    {
        meanpos += pp.calibratedPhotostations[i].location.shift * (1.0 / PSN);
    }
    double err = 0.0, cnt = 0.0;
    for (size_t i = 0; i < pois.size(); ++i)
	{
	    double mindist = 1e100;
	    for (int j = 0; j < PSN; ++j)
	        mindist = std::min(mindist, !(pois[i].worldPoint - pp.calibratedPhotostations[j].location.shift));
	    if (mindist < 30.0)
        {
            auto diff = !(pois[i].worldPoint - res[1][i].worldPoint);
            err += diff * diff;
            cnt += 1.0;
        }
		std::cout << pois[i].label << pois[i].worldPoint << " | " << res[0][i].worldPoint << " | " << res[1][i].worldPoint << ":" << (pois[i].worldPoint - res[1][i].worldPoint) << " /" << !(pois[i].worldPoint - res[1][i].worldPoint) << "\\ " << " / " << ((!(pois[i].worldPoint - res[1][i].worldPoint) / !(pois[i].worldPoint - meanpos))) * 100.0 << "%  ~ " << !(pois[i].worldPoint - meanpos) << "m //" << mindist << "m" << std::endl;
		for (size_t j = 0; j < pois[i].projections.size(); ++j)
		{
			std::cout << "\t\t" << ((char)('A' + pois[i].projections[j].photostationId)) << pois[i].projections[j].cameraId << ": " << pois[i].projections[j].projection << " : " << res[0][i].projections[j].projection << " (" << !(pois[i].projections[j].projection - res[0][i].projections[j].projection) << ") | " << res[1][i].projections[j].projection << " (" << !(pois[i].projections[j].projection - res[1][i].projections[j].projection) << ")" << "" << std::endl;
		}
		std::cout << std::endl;
	}
    std::cout << "30m new-style rmse error: " << std::sqrt(err / cnt) << std::endl;

    for (int i = 0; i < 6; ++i)
    {
        std::cout << i << ": FOCAL:" << job.photostation.cameras[i].intrinsics.focal
                               << pp.calibratedPhotostations[0].cameras[i].intrinsics.focal
                               << "(" << (job.photostation.cameras[i].intrinsics.focal[0] / pp.calibratedPhotostations[0].cameras[i].intrinsics.focal[0] - 1.0) * 100.0 << "%)"
                               << ": PRINCIPAL:" << job.photostation.cameras[i].intrinsics.principal
                               << pp.calibratedPhotostations[0].cameras[i].intrinsics.principal
                               << "(" << !(job.photostation.cameras[i].intrinsics.principal - pp.calibratedPhotostations[0].cameras[i].intrinsics.principal) << "px)"
                               << std::endl;
    }

    JSONSetter *setter2 = new JSONSetter("pp_final.json");
    setter2->visit(pp, "reconstruction data");
    delete setter2;
#if 0
    auto reproj = pp.projectToAll(pois_proj);
    storePois(reproj, "pois_all_proj.txt", "roof_1_", job);
#endif
    return 0;
}
#else
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

    ReconstructionFixtureScene rfs;
    std::vector<FixtureCamera*> cameras;
    std::vector<CameraFixture*> photostations;
    for (int i = 0; i < CP; ++i)
    {
        cameras.push_back(rfs.createCamera());
        cameras.rbegin()[0]->intrinsics = photostation.cameras[i].intrinsics;
        cameras.rbegin()[0]->extrinsics = photostation.cameras[i].extrinsics;
        cameras.rbegin()[0]->distortion = photostation.cameras[i].distortion;
        cameras.rbegin()[0]->nameId = photostation.cameras[i].nameId;
    }
    for (int i = 0; i < PSN; ++i)
    {
        photostations.push_back(rfs.createCameraFixture());
        for (int j = 0; j < CP; ++j)
        {
            rfs.addCameraToFixture(cameras[j], photostations[i]);
            rfs.images[WPP(photostations[i], cameras[j])] = images[i][j];
        }
        rfs.initializationData[photostations[i]].initializationType = PhotostationInitializationType::GPS;
        rfs.initializationData[photostations[i]].initData.shift = positions[psPrefixes[i] - 'A'];
        std::stringstream ss;
        ss << "SP" << ((char)('A' + i));
        rfs.fixtures[i]->name = ss.str();
    }
    rfs.placingQueue = photostations;

    PhotostationPlacer pp;
    pp.scene = &rfs;
    pp.fullRun();
    return 0;
}
#endif
