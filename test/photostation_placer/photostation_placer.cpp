/*
 * This test takes photostation model from calibrationJob and arranges copies
 * of this PS using GPS coordinates
 *
 * NOTE: this one is not finalized yet and is more "playground" than "application"
 * All files that you need are here
 * https://drive.google.com/folderview?id=0B1FS8EqRjHUrYjNNaDM5VzZUNlk&usp=sharing
 */
#include <fstream>
#include <regex>
#include <random>

#include "reconstructionSolver.h"
#include "jsonGetter.h"
#include "jsonSetter.h"
#include "pnpSolver.h"

#ifdef WITH_OPENCV
# include "openCvFileReader.h"
# include "openCvDescriptorExtractorWrapper.h"
# include "openCvFeatureDetectorWrapper.h"
# include "openCvDescriptorMatcherWrapper.h"
#endif

corecvs::Vector3dd convertVector(const corecvs::Vector3dd& geodesic)
{
    auto vec = geodesic;
    std::swap(vec[0], vec[1]);
#ifndef METERS
    return vec * 1e3;
#else
    return vec;
#endif
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
 //         if (camId == 0)
            projections.push_back(proj);
             std::cout << "POI: LABEL: " << label << ": CAM: " << filename << " (" << camId << "|" << psId << ")" << proj.projection << std::endl;

        }
        observation.projections = projections;
//      if (projections.size())
        if (!observation.updateable && projections.size() > 1)
        res.push_back(observation);
    }

    return res;

}

// XXX: here we parse only position part; angle data is rejected
//      we also make prefix uppercase since it is partially lower-case
//      in Measure_15
std::unordered_map<std::string, corecvs::Affine3DQ>
    parseGps(const std::string &filename
           , const corecvs::Matrix33 &axis = corecvs::Matrix33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0) )
{
    CORE_UNUSED(axis);

    std::ifstream ifs;
    ifs.open(filename, std::ios_base::in);
    if (!ifs)
    {
        std::cout << "GPS data file " << filename << " cannot be opened!" << std::endl;
        exit(0);
    }

    std::unordered_map<std::string, Affine3DQ> locations;
    std::string data;
    // Just prefix and 3 numbers delimited by spaces
    std::string prefix_regex = "([A-Za-z]+)",
                float_regex  = "([+-]?[0-9]+\\.?[0-9]*)",
                sep_regex    = "\\s+";
    std::regex re("^" + prefix_regex + sep_regex + float_regex + sep_regex + float_regex + sep_regex + float_regex +  "((\\s.*)?|$)");
    do
    {
        std::getline(ifs, data);
        if (data.size() && *data.rbegin() == '\r')
            data.resize(data.size() - 1);
        if (!data.size()) continue;

        std::smatch m;
        if (!std::regex_match(data, m, re) || m.size() < 5) // 4 tokens + final
        {
            std::cout << "REJECTED: " << data << std::endl;
            continue;
        }

        std::string key;
        double n, e, h;
        std::stringstream ss(data);
        ss >> key >> n >> e >> h;
        for (auto& c: key) c = toupper(c);
        std::cout << "Key: " << key << " geodesic stuff: " << n << " " << e << " " << h << std::endl;

        // XXX: note, that stuff valid only for current proto, somewhere we should add switch for it
        CameraLocationData location;
#ifndef METERS
        location.position[0] = e * 1e3;
        location.position[1] = n * 1e3;
        location.position[2] = h * 1e3;
#else
        location.position[0] = e;
        location.position[1] = n;
        location.position[2] = h;
#endif
        location.orientation = corecvs::Quaternion(0.0, 0.0, 0.0, 1.0);

        locations[key] = location.toAffine3D();

        std::cout << "Key: " << key << " our stuff: " << location.position << " " << location.orientation << std::endl;
    } while(ifs);
    return locations;
}

void run_pois(int camIdOffset, bool distorted, bool filter, bool forceGps)
{
    // 0. Reposition camera
    // 1. Read & filter pois
    // 2. Read GT
    // 3. Run solver
    // 4. Compute reprojection & distance error

    /*
     * First, we need to read calibration data
     * and scale it
     */
    auto map = parseGps("ps_coords_m15.txt");
    CalibrationJob jobC;
    JSONGetter getter("calibration.json");
    getter.visit(jobC, "job");

    /*
     * Repositioning camera into E-N plane by first 6 cameras
     */
    std::vector<int> perm;
    for (int i = 0; i < jobC.photostation.cameras.size(); ++i)
        perm.push_back(i);
    std::sort(perm.begin(), perm.end(), [&](const int &a, const int &b) { return jobC.photostation.cameras[a].nameId < jobC.photostation.cameras[b].nameId; });

    auto cameras = jobC.photostation.cameras;
    auto setups = jobC.calibrationSetups;
    auto observations = jobC.observations;

    for (auto& s: jobC.calibrationSetups)
        for (auto& v: s)
            v.cameraId = perm[v.cameraId];
    for (int i = 0; i < cameras.size(); ++i)
    {
        jobC.observations[i] = observations[perm[i]];
        jobC.photostation.cameras[i] = cameras[perm[i]];
    }

    jobC.photostation.location.shift = corecvs::Vector3dd(0.0, 0.0, 0.0);
    jobC.photostation.location.rotor = jobC.photostation.cameras[0].extrinsics.orientation;
    corecvs::Vector3dd meanShift(0.0, 0.0, 0.0);
    for (int i = 0; i < jobC.photostation.cameras.size(); ++i)
    {
        jobC.photostation.cameras[i] = jobC.photostation.getRawCamera(i);
    }
#if 1
    jobC.photostation.location.rotor = corecvs::Quaternion::FromMatrix(
            corecvs::Matrix33(
               1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0
                ));
    jobC.photostation.location.shift = corecvs::Vector3dd(0.0, 0.0, 0.0);
    for (int i = 0; i < jobC.photostation.cameras.size(); ++i)
    {
        jobC.photostation.cameras[i] = jobC.photostation.getRawCamera(i);
    }
#endif
    jobC.photostation.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    for (int i = 0; i < jobC.photostation.cameras.size(); ++i)
    {
        if (i < 6)
            meanShift += jobC.photostation.getRawCamera(i).extrinsics.position;
    }
    jobC.photostation.location.rotor = corecvs::Quaternion(0.0, 0.0, 0.0, 1.0);
    jobC.photostation.location.shift = -meanShift * (1.0 / 6.0);
//    meanShift = corecvs::Vector3dd(0, 0, 0);
    for (int i = 0; i < jobC.photostation.cameras.size(); ++i)
    {
        jobC.photostation.cameras[i] = jobC.photostation.getRawCamera(i);
    }
    jobC.photostation.location.shift = corecvs::Vector3dd(0, 0, 0);
    meanShift = corecvs::Vector3dd(0, 0, 0);
    for (int i = 0; i < jobC.photostation.cameras.size(); ++i)
        if (i < 6)
            meanShift += jobC.photostation.getRawCamera(i).extrinsics.position * (1.0 / 6.0);
    std::cout << "MS: " << meanShift << std::endl;
    jobC.photostation.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    jobC.photostation.location.shift = corecvs::Vector3dd(0, 0, 0);

    JSONSetter* setter = new JSONSetter("calibration_mod.json");
    for (int i = 0; i < jobC.calibrationSetupLocations.size(); ++i)
    {
       jobC.calibrationSetupLocations[i].position = corecvs::Vector3dd(0, 0, 0);
       jobC.calibrationSetupLocations[i].orientation = corecvs::Quaternion(0, 0, 0, 1);
    }
    setter->visit(jobC, "job");
    delete setter;


    auto pps = jobC.photostation;
#ifdef METERS
    for (auto& cam: jobC.photostation.cameras)
        cam.extrinsics.position *= 1e-3;
#endif
    double maxAngleError = 1.0;

    ReconstructionJob rec;
    rec.calibrationData = jobC;
    rec.fill(map, map.size());
    for (auto& p: rec.scene.photostations)
    {
        std::cout << "CHECKPS: " << p.location.shift << " : " << p.location.rotor << std::endl;
    }

    /*
     * Next step is parsing pois (and scaling/transforming them from geodesic
     * system)
     */
    rec.scene.pointObservations = parsePois(jobC, "pois_m15.txt",camIdOffset, false, true);
    std::cout << "images = {" << std::endl;
    for (auto &o : rec.scene.pointObservations)
    {
        std::cout << "{";
        for (auto &p: o.projections)
        {
            std::cout << "'/hdd_4t/data/roof_v1/roof_1_SP" << ((char)('A' + p.photostationId)) << p.cameraId << "_0deg.jpg'";
            if (&p == &(*o.projections.rbegin()))
                std::cout << "}," << std::endl;
            else
                std::cout << ", ";
        }
    }
    std::cout << "};" << std::endl;
    std::cout << "projections = {" << std::endl;
    for (auto &o : rec.scene.pointObservations)
    {
        std::cout << "{" << std::endl;
        for (auto &p: o.projections)
        {
            std::cout << "[" << p.projection[0] << " " << p.projection[1] << "]";
            if (&p == &(*o.projections.rbegin()))
                std::cout << "}," << std::endl;
            else
                std::cout << ", ";
        }
    }
    std::cout << "};" << std::endl;
    rec.scene.pointObservations = parsePois(jobC, "pois_m15.txt",camIdOffset, distorted, true);
    std::cout << "images = {" << std::endl;
    for (auto &o : rec.scene.pointObservations)
    {
        std::cout << "{";
        for (auto &p: o.projections)
        {
            std::cout << "'/hdd_4t/data/roof_v1/roof_1_SP" << ((char)('A' + p.photostationId)) << p.cameraId << "_0deg_undist.jpg'";
            if (&p == &(*o.projections.rbegin()))
                std::cout << "}," << std::endl;
            else
                std::cout << ", ";
        }
    }
    std::cout << "};" << std::endl;
    std::cout << "projections = {" << std::endl;
    for (auto &o : rec.scene.pointObservations)
    {
        std::cout << "{";
        for (auto &p: o.projections)
        {
            std::cout << "[" << p.projection[0] << " " << p.projection[1] << "]";
            if (&p == &(*o.projections.rbegin()))
                std::cout << "}," << std::endl;
            else
                std::cout << ", ";
        }
    }
    std::cout << "};" << std::endl;
    /*
     * Now we select only good points
     */

    if (filter)
    {
        std::mt19937 rng;
        for (int i = 0; i < rec.scene.photostations.size(); ++i)
        {
            std::vector<std::tuple<int, corecvs::Vector2dd, corecvs::Vector3dd>> pspa;
            for (auto &o: rec.scene.pointObservations)
            {
                for (auto&p: o.projections)
                {
                    if (p.photostationId == i && !o.updateable)
                    {
                        pspa.emplace_back(p.cameraId, p.projection, o.worldPoint);
                    }
                }
            }

            double bhScore = 1e100;
            corecvs::Affine3DQ bh;
            for (int iii = 0; iii < 10000; ++iii)
            {
                decltype(pspa) psp;
                int N = pspa.size();
                psp.resize(4);
                int ids[4];
                int idx = 0;
                ids[0] = rng() % N;
                while (idx < 4)
                {
                    int idd = rng() % N;
                    bool isValid = true;
                    for (int ii = 0; ii < idx; ++ii)
                        if (ids[ii] == idd)
                            isValid = false;
                    if (isValid)
                        ids[idx++] = idd;
                }
                for (int kk = 0; kk < 4; ++kk)
                {
                    psp[kk] = pspa[ids[kk]];
                }

            std::vector<corecvs::Vector3dd> pts, dirs,oris;
            for (auto& t: psp)
            {
                corecvs::Vector3dd dir = jobC.photostation.getRawCamera(std::get<0>(t)).rayFromPixel(std::get<1>(t)).a.normalised();
                corecvs::Vector3dd ori = jobC.photostation.getRawCamera(std::get<0>(t)).extrinsics.position;
                corecvs::Vector3dd pt  = std::get<2>(t);
                pts.push_back(pt);
                oris.push_back(ori);
                dirs.push_back(dir);
            }
            auto hyp = corecvs::PNPSolver::solvePNP(oris, dirs, pts);
            for (auto& h: hyp)
            {
                auto ps = jobC.photostation;
                ps.location = h;
                double score = 0.0;
                for (auto& t: pspa)
                {
                    auto proj = ps.project(std::get<2>(t), std::get<0>(t));
                    auto cam = ps.getRawCamera(std::get<0>(t));
                    auto dp = (cam.rayFromPixel(std::get<1>(t)).a).normalised();
                    auto diff = std::get<1>(t) - proj;
                    double angle = std::abs((std::get<2>(t) - cam.extrinsics.position).angleTo(dp)) * 180.0 / M_PI;
                    if (angle < maxAngleError && cam.isVisible(std::get<2>(t)))
                        score -= 1.0;// diff & diff;
                }
                if (score < bhScore)
                {
                    bhScore = score;
                    bh = h;
                }
            }
            }
            if (bhScore <= -3)
            {
                if (!forceGps)
                    rec.scene.photostations[i].location = bh;
                else
                    rec.scene.photostations[i].location.rotor = bh.rotor;
            }
        }
        std::cout << "ANGLES: " << std::endl;
        for (auto&o : rec.scene.pointObservations)
        {
            std::cout << "AE: " <<  o.label << ": ";
            for (auto&p : o.projections)
            {
                auto cam = rec.scene.photostations[p.photostationId].getRawCamera(p.cameraId);
                auto dp = (cam.rayFromPixel(p.projection)).a;
                double angle = (o.worldPoint - cam.extrinsics.position).angleTo(dp) * 180.0 / M_PI;
                std::cout << "[SP" << ((char)('A' + p.photostationId)) << p.cameraId << " : " << angle << "]";
            }
            std::cout << std::endl;
        }
        decltype(rec.scene.pointObservations) obsNew;
        for (auto&o : rec.scene.pointObservations)
        {
            std::cout << "AE: " <<  o.label << ": ";
            auto on = o;
            on.projections.clear();
            for (auto&p : o.projections)
            {
                auto cam = rec.scene.photostations[p.photostationId].getRawCamera(p.cameraId);
                auto dp = (cam.rayFromPixel(p.projection)).a;
                double angle = (o.worldPoint - cam.extrinsics.position).angleTo(dp) * 180.0 / M_PI;
                std::cout << "[SP" << ((char)('A' + p.photostationId)) << p.cameraId << " : " << angle << "]";
                if (std::abs(angle) < maxAngleError && cam.isVisible(o.worldPoint))
                    on.projections.push_back(p);
            }
            if (on.projections.size() > 1)
                obsNew.push_back(on);
            std::cout << std::endl;
        }
        rec.scene.pointObservations = obsNew;
    }
    // And refine positions
#if 1
    for (int i = 0; i < rec.scene.photostations.size(); ++i)
    {
        std::vector<std::tuple<int, corecvs::Vector2dd, corecvs::Vector3dd>> pspa;
        for (auto &o: rec.scene.pointObservations)
        {
            for (auto&p: o.projections)
            {
                if (p.photostationId == i && !o.updateable)
                {
                    pspa.emplace_back(p.cameraId, p.projection, o.worldPoint);
                }
            }
        }
        if (pspa.size() < 3)
            continue;

        double bhScore = 1e100;
        corecvs::Affine3DQ bh;
        std::vector<corecvs::Vector3dd> pts, dirs,oris;
        auto psp = pspa;
        for (auto& t: psp)
        {
            corecvs::Vector3dd dir = jobC.photostation.getRawCamera(std::get<0>(t)).rayFromPixel(std::get<1>(t)).a.normalised();
            corecvs::Vector3dd ori = jobC.photostation.getRawCamera(std::get<0>(t)).extrinsics.position;
            corecvs::Vector3dd pt  = std::get<2>(t);
            pts.push_back(pt);
            oris.push_back(ori);
            dirs.push_back(dir);
        }
        auto hyp = corecvs::PNPSolver::solvePNP(oris, dirs, pts);
        int cnt = 0;
        for (auto& h: hyp)
        {
            auto ps = jobC.photostation;
            ps.location = h;
            double score = 0.0;
            int cc = 0;
            for (auto& t: pspa)
            {
                auto proj = ps.project(std::get<2>(t), std::get<0>(t));
                auto cam = ps.getRawCamera(std::get<0>(t));
                auto dp = (cam.rayFromPixel(std::get<1>(t)).a).normalised();
                auto diff = std::get<1>(t) - proj;
                double angle = std::abs((std::get<2>(t) - cam.extrinsics.position).angleTo(dp)) * 180.0 / M_PI;
                score += diff & diff;
            }
            std::cout << "SP" << (char)('A' + i) << " : " << h.shift;
            h.rotor.printAxisAndAngle();
            std::cout << "SC: " << std::sqrt(score / pspa.size()) << std::endl;
            if (score < bhScore)
            {
                bhScore = score;
                bh = h;
            }
        }
        if (pspa.size() > 2)
        {
            if (!forceGps)
               rec.scene.photostations[i].location = bh;
            else
                rec.scene.photostations[i].location.rotor = bh.rotor;
        }
    }
#endif
    /*
     * Now we run solver 2 times: with geometrical error,
     * and continue optimization with graphical (e.g. reprojection)
     * error
     */
    // Geometrical error optimization

    rec.solve(true);
    std::vector<double> errors;
    rec.scene.computeReprojectionErrors(errors);
    double ssq = 0.0;
    for (uint i = 0; i < errors.size(); ++i)
        ssq += errors[i] * errors[i];
    std::cout << "ERROR: " << std::sqrt(ssq / (errors.size() / 2)) << std::endl;
    corecvs::Vector3dd meanPos(0.0, 0.0, 0.0);
    for (auto&p : rec.scene.photostations)
    {
        meanPos += p.location.shift;
    }
    meanPos = meanPos * (1.0 / rec.scene.photostations.size());
    for (auto&o : rec.scene.pointObservations)
    {
        auto wp = o.worldPoint;
        auto wp2 = rec.scene.backProject(o.projections, o.updateable);
        std::cout << o.label << " GT: " << wp << " OBS: " << wp2 << " (" << (!(wp - wp2)/1e3) << ")" <<  ": " << (!(wp-wp2)/!(wp - meanPos)) * 100.0 << "% " << std::endl;
    }
    std::cout << "ANGLES: " << std::endl;
    for (auto&o : rec.scene.pointObservations)
    {
        std::cout << "AE: " <<  o.label << ": ";
        for (auto&p : o.projections)
        {
            auto cam = rec.scene.photostations[p.photostationId].getRawCamera(p.cameraId);
            auto dp = (cam.rayFromPixel(p.projection)).a;
            double angle = (o.worldPoint - cam.extrinsics.position).angleTo(dp) * 180.0 / M_PI;
            std::cout << "[SP" << ((char)('A' + p.photostationId)) << p.cameraId << " : " << angle << "]";
        }
        std::cout << std::endl;
    }

    std::cout << "GT:" << std::endl;
    rec.solve(false);

    /*
     * Now it's time for weird output of errors
     */
    rec.scene.computeReprojectionErrors(errors);
    ssq = 0.0;
    for (uint i = 0; i < errors.size(); ++i)
        ssq += errors[i] * errors[i];
    // Final reprojection error
    std::cout << "ERROR: " << std::sqrt(ssq / (errors.size() / 2)) << std::endl;
    meanPos = corecvs::Vector3dd(0.0, 0.0, 0.0);
    // Individual reprojection errors
    for (auto&o : rec.scene.pointObservations)
    {
        std::cout << "RE: " <<  o.label << ": ";
        for (auto&p : o.projections)
        {
            auto diff = rec.scene.computeReprojectionError(p.photostationId, p.cameraId, o.worldPoint, p.projection);
            std::cout << "[SP" << ((char)('A' + p.photostationId)) << p.cameraId << " : " << diff << " (" << !diff << ")]";
        }
        std::cout << std::endl;
    }
    std::cout << "ANGLES: " << std::endl;
    for (auto&o : rec.scene.pointObservations)
    {
        std::cout << "AE: " <<  o.label << ": ";
        for (auto&p : o.projections)
        {
            auto cam = rec.scene.photostations[p.photostationId].getRawCamera(p.cameraId);
            auto dp = (cam.rayFromPixel(p.projection)).a;
            double angle = (o.worldPoint - cam.extrinsics.position).angleTo(dp) * 180.0 / M_PI;
            std::cout << "[SP" << ((char)('A' + p.photostationId)) << p.cameraId << " : " << angle << "]";
        }
        std::cout << std::endl;
    }

    /*
     * Two methods of total (distance) error calculation
     */
    for (auto&p : rec.scene.photostations)
    {
        meanPos += p.location.shift;
    }
    meanPos = meanPos * (1.0 / rec.scene.photostations.size());
    double meanGt = 0.0, gtCnt = 0.0;
    // Relative error as difference of expected vs observed divided
    // by distance to expected from mean point
    for (auto&o : rec.scene.pointObservations)
    {
        auto wp = o.worldPoint;
        auto wp2 = rec.scene.backProject(o.projections, o.updateable);
        std::cout << o.label << " GT: " << wp << " OBS: " << wp2 << " (" << (!(wp - wp2)) << ")" <<  ": " << "[" << !(wp - meanPos) << " | " << (!(wp-wp2)/!(wp - meanPos)) * 100.0 << "% ]" << std::endl;
        meanGt += (!(wp-wp2)/!(wp - meanPos));
        gtCnt += 1.0;
    }
    std::cout << std::endl;
    // Relative error as difference in POI<->POI distances
    // divided by distance from mean point to mean point between POIs
    double meanGtr = 0.0, gtrCnt = 0.0;
    for (auto&o  : rec.scene.pointObservations)
    {
        std::cout << "REPERROR: " << o.label << " ";
        for (auto&p : o.projections)
        {
            auto wp = rec.scene.backProject(o.projections, o.updateable);
            auto pp = rec.scene.photostations[p.photostationId].project(o.worldPoint, p.cameraId);
            auto pp2= rec.scene.photostations[p.photostationId].project(wp, p.cameraId);
            auto diff1 = pp - p.projection;
            auto diff2 = pp2- p.projection;
            std::cout << "SP" << (char)('A' + p.photostationId) << p.cameraId << ": " << !diff1 << " | " << !diff2;
        }
        std::cout << std::endl;
    }

    for (auto&o2 : rec.scene.pointObservations)
    {
        for (auto&o : rec.scene.pointObservations)
        {
            auto wp1 = o.worldPoint;
            auto wp2 = o2.worldPoint;
            if (wp1==wp2) continue;

            auto wp12 = rec.scene.backProject(o.projections, o.updateable);
            auto wp22 = rec.scene.backProject(o2.projections, o2.updateable);
            std::cout << "GTR: " << o.label << "<->" << o2.label << " " << std::abs(!(wp12 - wp22)-!(wp1 - wp2))/!(wp1/2.0 + wp2/2.0 - meanPos)  << "; ";
            meanGtr += std::abs(!(wp12 - wp22)-!(wp1 - wp2))/!(wp1/2.0 + wp2/2.0 - meanPos);
            gtrCnt += 1.0;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    /*
     * Final table output
     */
    std::cout << "|_. POI|_. Ground truth|_. Observed|_. Error|_. Distance to mean point|_. Error w.r.t distance to photostations|";
    for (int i = 0; i < rec.scene.photostations.size(); ++i)
        std::cout << "_.SP" << ((char)('A' + i)) << " (RP)|_.SP" << ((char)('A' + i)) << " (WP)|";
    std::cout << std::endl;
    for (auto& o: rec.scene.pointObservations)
    {
        auto wp = o.worldPoint;
        auto rp = rec.scene.backProject(o.projections, o.updateable);
        std::cout << "|" << o.label << "|" << wp << "|" << rp << "|" << !(wp - rp) << "|" << !(wp - meanPos) << "|" << (!(wp - rp)/!(wp - meanPos)) * 100.0 << "%|";
        for (int pi = 0; pi < rec.scene.photostations.size(); ++pi)
        {

            bool out = false;
            for (auto& p: o.projections)
            {
                if (p.photostationId != pi)
                    continue;
                auto pp = rec.scene.photostations[p.photostationId].project(wp, p.cameraId);
                auto pp2= rec.scene.photostations[p.photostationId].project(rp, p.cameraId);
                auto diff1 = pp - p.projection;
                auto diff2 = pp2- p.projection;
                std::cout << !diff1 << "|" << !diff2 << "|";
                out = true;
                break;
            }
            if (!out) std::cout << "NA|NA|";
        }
        std::cout << std::endl;
    }
    // And here we get mean error in %
    std::cout << "GTRM: " << meanGtr / gtrCnt * 100.0 << std::endl;
    std::cout << "GTM: " << meanGt / gtCnt * 100.0 << std::endl;

    std::cout << "Focals:" << std::endl;
    for (int i = 0; i < jobC.photostation.cameras.size(); ++i)
    {
        std::cout << "Camera " << jobC.photostation.cameras[i].nameId << ": ";
        auto diff =  jobC.photostation.cameras[i].intrinsics.focal - rec.scene.photostations[0].cameras[i].intrinsics.focal;
        std::cout << diff << (!(diff)/!jobC.photostation.cameras[i].intrinsics.focal) * 100.0 << "%" << std::endl;
    }
    std::cout << "CxCy" << std::endl;
    for (int i = 0; i < jobC.photostation.cameras.size(); ++i)
    {
        std::cout << "Camera " << jobC.photostation.cameras[i].nameId << ": ";
        auto diff =  jobC.photostation.cameras[i].intrinsics.principal - rec.scene.photostations[0].cameras[i].intrinsics.principal;
        std::cout << diff << (!(diff)/!jobC.photostation.cameras[i].intrinsics.principal) * 100.0 << "%" << std::endl;
    }
    std::cout << "Orientations:" << std::endl;
    for (int i = 0; i < jobC.photostation.cameras.size(); ++i)
    {
        std::cout << "Camera " << jobC.photostation.cameras[i].nameId << ": ";
        auto diff =  jobC.photostation.cameras[i].extrinsics.orientation ^ rec.scene.photostations[0].cameras[i].extrinsics.orientation;
        diff.printAxisAndAngle();
        std::cout << std::endl;
    }
    std::cout << "Positions" << std::endl;
    for (int i = 0; i < jobC.photostation.cameras.size(); ++i)
    {
        std::cout << "Camera " << jobC.photostation.cameras[i].nameId << ": ";
        auto diff =  jobC.photostation.cameras[i].extrinsics.position - rec.scene.photostations[0].cameras[i].extrinsics.position;
        std::cout << diff << (!(diff)/!jobC.photostation.cameras[i].extrinsics.position) * 100.0 << "%" << std::endl;
    }
}

int main(int argc, char **argv)
{
    init_opencv_detectors_provider();
    init_opencv_matchers_provider();
    init_opencv_reader_provider();
    init_opencv_descriptors_provider();
    init_opencv_reader_provider();

    bool m15 = false;
    bool filter = false;
    bool forceGps = true;
    if (argc > 1)
    {
        std::string arg(argv[1]);
        if (arg == "m15")
        {
            m15 = true;
        }

        std::cout << "First argument is " << arg << ", running in " << (m15 ? "m15" : "m16") << " mode" << std::endl;

    }
    if (argc > 2)
    {
        std::string arg(argv[2]);
        if (arg == "filter")
        {
            filter = true;
        }

        std::cout << "Second argument is " << arg << ", " << (filter ? "will" : "will not") << " filter pois" << std::endl;
    }
    if (argc > 3)
    {
        std::string arg(argv[2]);
        if (arg == "forceGps")
        {
            forceGps = true;
        }

        std::cout << "Third argument is " << arg << ", " << (forceGps ? "will" : "will not") << " enforce GPS data" << std::endl;
    }
    std::cout << (m15 ? "m15" : "m16") << " mode is selected" << std::endl;
    std::cout << (filter ? "will" : "will not") << " filter pois" << std::endl;
    std::cout << (filter ? "will" : "will not") << " force GPS" << std::endl;


    // M15 settings
    bool distorted = false;
    int camIdOffset = 3;
    // M16 settings
    if (!m15)
    {
        camIdOffset = 10;
        distorted = true;
    }
    run_pois(camIdOffset, distorted, filter, forceGps);
}
