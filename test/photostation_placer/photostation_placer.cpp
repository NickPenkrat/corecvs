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

#include "reconstructionSolver.h"
#include "jsonGetter.h"

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


std::vector<PointObservation__> parsePois(const std::string &filename, bool m15 = true)
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
        }
        assert(lstr.size());
        std::stringstream ss(lstr);
    
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

            int psId = filename[2] - 'A';

            int camId = m15 ?
                (filename[3] - '0')
                :((filename[3] - '0') * 10 + filename[4] - '0') - 1;
            PointProjection proj;
            proj.projection = corecvs::Vector2dd(x, y)-corecvs::Vector2dd(0.5, 0.5);
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
//      location.position = axis * location.position;
        double phi=0.0;//-0.22*M_PI / 180.0;

        location.orientation = 
#if 0
            corecvs::Quaternion(0.0, 0.0, 0.0, 1.0);//corecvs::Quaternion(0.5, 0.5, 0.5, 0.5);//corecvs::Quaternion(sin(M_PI / 4.0), 0.0, 0.0, cos(M_PI / 4.0));
#else
        corecvs::Quaternion(0,0,sin(phi),cos(phi)) ^ corecvs::Quaternion(0.5, 0.5, 0.5, 0.5);//onjugated;
#endif

        locations[key] = location.toAffine3D();

        std::cout << "Key: " << key << " our stuff: " << location.position << " " << location.orientation << std::endl;
    } while(ifs);
    return locations;
}

void run_m15_pois()
{
    // 1. Read pois
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
    auto pps = jobC.photostation;
#ifdef METERS
    for (auto& cam: jobC.photostation.cameras)
        cam.extrinsics.position *= 1e-3;
#endif

    ReconstructionJob rec;
    rec.calibrationData = jobC;
    rec.fill(map);
    for (auto& p: rec.scene.photostations)
    {
        std::cout << "CHECKPS: " << p.location.shift << " : " << p.location.rotor << std::endl;
    }

    /*
     * Next step is parsing pois (and scaling/transforming them from geodesic
     * system)
     */
    rec.scene.pointObservations = parsePois("pois_m15.txt", true);

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
    // And here we get mean error in %
    std::cout << "GTRM: " << meanGtr / gtrCnt * 100.0 << std::endl;
    std::cout << "GTM: " << meanGt / gtCnt * 100.0 << std::endl;
}

int main(int argc, char **argv)
{
    init_opencv_detectors_provider();
    init_opencv_matchers_provider();
    init_opencv_reader_provider();
    init_opencv_descriptors_provider();
    init_opencv_reader_provider();


    std::string filenameCalibration = "calibration.json";
    std::string filenameGPS         = "gps.txt";
    std::string filenamePly         = "mesh.ply";

    if (argc > 1)
    {
        filenameCalibration = std::string(argv[1]);
    }
    if (argc > 2)
    {
        filenameGPS = std::string(argv[2]);
    }
    if (argc > 3)
    {
        filenamePly = std::string(argv[3]);
    }

    std::cout << "Reading calibration from " << filenameCalibration << std::endl <<
                 "Reading GPS data from " << filenameGPS << std::endl <<
                 "Saving mesh to " << filenamePly << std::endl;


    run_m15_pois();
}
