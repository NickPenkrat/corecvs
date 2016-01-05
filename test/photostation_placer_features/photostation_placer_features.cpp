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
                proj.projection = calibration.corrections[camId].map(proj.projection[1], proj.projection[0]);
            }
            proj.cameraId = camId;
            proj.photostationId = psId;
//    if (psId < 5)
            projections.push_back(proj);
             std::cout << "POI: LABEL: " << label << ": CAM: " << filename << " (" << camId << "|" << psId << ")" << proj.projection << std::endl;

        }
        observation.projections = projections;
//      if (projections.size())
//        if (!observation.updateable)
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
            auto correction = calibration.corrections[p.cameraId];
            auto projection = correction.invMap(p.projection[1], p.projection[0]);
            ofs << "roof_v1_SP" << ((char)('A' + p.photostationId)) << p.cameraId << "_0deg.jpg" << " " << projection[0] << " " << projection[1] << std::endl;
        }
    }
}

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
	corecvs::Vector3dd mp(0, 0, 0);
	for (int i = 0; i < 9; ++i)
	{
		mp += positions[i] * (1.0 / 9.0);
//positions[i][2] = 0;
	}

    PhotostationPlacer pp;
   const int PSN = 9;
    const int CP = 6;
    char psPrefixes[9] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};
    
    std::string prefix = "/hdd_4t/data/roof_v1/roof_1_SP";
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
    job.prepareAllRadialCorrections();
    auto pois = parsePois(job, "pois_m15.txt",10, true, true);
    auto pois_proj = parsePois(job, "pois_all.txt",10, true, true);

	for (auto &o: pois)
	{
		std::cout << o.label << " " << o.worldPoint << " " << o.worldPoint - mp << " " << !(o.worldPoint - mp) << std::endl;
	}

    photostation = job.photostation;
    for (int i = 0; i < photostation.cameras.size(); ++i)
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
        corecvs::Photostation ps = pp.calibratedPhotostations[i];
        for (int j = 0; j < ps.cameras.size(); ++j)
            ps.cameras[j].extrinsics.position *= 1e3;
        ps.location.shift -= meanpos;
        ps.location.shift *= 1e3;

        CalibrationHelpers().drawPly(meshres, ps, 50.0);
    }

    pp.buildTracks(0, 1, 2);
	std::cout << "TR: " << pp.tracks.size() << " " << pp.getReprojectionCnt() / 2 << std::endl;
	pp.fit();
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
        corecvs::Photostation ps = pp.calibratedPhotostations[i];
        for (int j = 0; j < ps.cameras.size(); ++j)
            ps.cameras[j].extrinsics.position *= 1e3;
        ps.location.shift -= meanpos;
        ps.location.shift *= 1e3;
		std::cout << ps.location.shift << std::endl;
        
        CalibrationHelpers().drawPly(meshres3, ps, 50.0);
    }
	std::cout << "TRACKS: " << pp.tracks.size() << std::endl;
    for(auto&p : pp.tracks)
        meshres3.addPoint((p.worldPoint-meanpos) * 1e3);
    meshres3.dumpPLY("triples_tri.ply");

	for (int i = 3; i < PSN; ++i)
	{
//        pp.fit();
        pp.appendPs();
        for (int j = 0; j < i; ++j)
            for (int k = j + 1; k < i; ++k)
                pp.buildTracks(j, k, i);
	}
	pp.fit(PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS);

    Mesh3D meshres4;
    meshres4.switchColor(true);
    meanpos = corecvs::Vector3dd(0, 0, 0);
    for (int i = 0; i < PSN; ++i)
    {
        meanpos += pp.calibratedPhotostations[i].location.shift * (1.0 / PSN);
    }
    for (int i = 0; i < PSN; ++i)
    {
        corecvs::Photostation ps = pp.calibratedPhotostations[i];
        for (int j = 0; j < ps.cameras.size(); ++j)
            ps.cameras[j].extrinsics.position *= 1e3;
        ps.location.shift -= meanpos;
        ps.location.shift *= 1e3;
		std::cout << ps.location.shift << std::endl;

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

    auto res = pp.verify(pois);
    for (int i = 0; i < pois.size(); ++i)
	{
		std::cout << pois[i].label << pois[i].worldPoint << " | " << res[0][i].worldPoint << " | " << res[1][i].worldPoint << ":" << (pois[i].worldPoint - res[1][i].worldPoint) << " / " << ((!(pois[i].worldPoint - res[1][i].worldPoint) / !(pois[i].worldPoint - meanpos))) * 100.0 << "%  ~ " << !(pois[i].worldPoint - meanpos) << "m" << std::endl;
		for (int j = 0; j < pois[i].projections.size(); ++j)
		{
			std::cout << "\t\t" << ((char)('A' + pois[i].projections[j].photostationId)) << pois[i].projections[j].cameraId << ": " << pois[i].projections[j].projection << " : " << res[0][i].projections[j].projection << " (" << !(pois[i].projections[j].projection - res[0][i].projections[j].projection) << ") | " << res[1][i].projections[j].projection << " (" << !(pois[i].projections[j].projection - res[1][i].projections[j].projection) << ")" << "" << std::endl;
		}
		std::cout << std::endl;
	}
    
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

    auto reproj = pp.projectToAll(pois_proj);
    storePois(reproj, "pois_all_proj.txt", "roof_1_", job);
    return 0;
}
