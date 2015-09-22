/*
 * This test takes photostation model from calibrationJob and arranges copies
 * of this PS using GPS coordinates
 */
#include <vector>
#include <string>
#include <sstream>
#include <regex>
#include <fstream>
#include <unordered_map>
#include <cstdio>
#include <algorithm>

#include "calibrationJob.h"
#include "jsonSetter.h"
#include "jsonGetter.h"

#include "imageKeyPoints.h"

#include "vector3d.h"
#include "vector2d.h"


#if 0
struct PointProjection;
struct PointObservation;

struct PointProjection
{
    corecvs::Vector2dd projection;
    int photostationId;
    int cameraId;
    int featureId;

    template<typename V>
    void accept(V &visitor)
    {
        visitor.visit(projection,     corecvs::Vector2dd(0.0, 0.0), "projection");
        visitor.visit(photostationId, 0,                            "photostationId");
        visitor.visit(cameraId,       0,                            "cameraId");
        visitor.visit(featureId,      -1,                           "featureId");
    }
};

struct PointObservation__
{
    corecvs::Vector3dd worldPoint;

    std::vector<PointProjection> projections;

    template<typename V>
    void accept(V &visitor)
    {
        visitor.visit(worldPoint,  corecvs::Vector3dd(0.0, 0.0, 0.0), "worldPoint");
        visitor.visit(projections, "projections"); 
    }
};
struct CameraObservation
{
    std::string sourceFileName;
    std::string undistortedFileName;
    std::string descriptorFileName;

    std::vector<KeyPoint> keyPoints;

    template<typename V>
    void accept(V &visitor)
    {
        visitor.visit(sourceFileName,      std::string(""), "sourceFileName");
        visitor.visit(undistortedFileName, std::string(""), "undistortedFileName");
        visitor.visit(descriptorFileName,  std::string(""), "descriptorFileName");

        visitor.visit(keyPoints, "keyPoints");
    }
};

struct ReconstructionParameters
{
    DetectorType   detector   = "SURF";
    DescriptorType descriptor = "SURF";
    MatcherType    matcher    = "ANN";

    template<typename V>
    void accept(V &visitor)
    {
        visitor.visit(detector,   "SURF", "detector");
        visitor.visit(descriptor, "SURF", "descriptor");
        visitor.visit(matcher,    "ANN",  "matcher");
    }
};


struct MultiPhotostationScene
{
    std::vector<PointObservation__> pointObservations;
    std::vector<Photostation> photostations;
    std::vector<std::vector<CameraObservation> > cameraObservations;

//    std::vector<PointObservation> pointObservations;


#if 0
    void readParams (const double[] data);
    void writeParams(double[] data);
#endif

    corecvs::Vector2dd computeReprojectionError (int ps, int cam, const corecvs::Vector3dd &point, const corecvs::Vector2dd &expected)
    {
        return project(ps, cam, point) - expected;
    }
    void   computeReprojectionErrors(int ps, int cam, std::vector<double> &errors)
    {
        errors.clear();
        for (auto& p: pointObservations)
            for (auto& c: p.projections)
            {
               if (ps == c.photostationId && cam == c.cameraId)
               {
                   auto e = computeReprojectionError(ps, cam, p.worldPoint, c.projection);
                   errors.push_back(e[0]);
                   errors.push_back(e[1]);
               }
           }
    }
    void   computeReprojectionErrors(std::vector<double> &errors)
    {
        int M = photostations.size();
        errors.clear();
        for (int i = 0; i < M; ++i)
        {
            int N = photostations[i].cameras.size();
            for (int j = 0; j < N; ++j)
            {
                std::vector<double> err;
                computeReprojectionErrors(i, j, err);
                errors.insert(errors.end(), err.begin(), err.end());
            }
        }
    }
    corecvs::Vector2dd project(int ps, int cam, const corecvs::Vector3dd &point)
    {
        return photostations[ps].project(point, cam);
    }
    corecvs::Vector3dd backProject(std::vector<std::pair<std::pair<int, int>, corecvs::Vector2dd>> &points)
    {
        return corecvs::Vector3dd(0.0, 0.0, 0.0);
    }

    int getPhotostationCount() const
    {
        assert(photostations.size() == cameraObservations.size());
        for (size_t i = 0; i < photostations.size())
            assert(photostations[i].cameras.size() == cameraObservations[i].size());
        return photostations.size();
    }

    template<typename V>
    void accept(V &visitor)
    {
        visitor.visit(pointObservations,  "pointObservations");
        visitor.visit(photostations,      "photostations");
        visitor.visit(cameraObservations, "cameraObservations");
    }
};

struct ReconstructionJob : ReconstructionParameters
{
    CalibrationJob         calibrationData;
    MultiPhotostationScene scene;

    void undistortAll(bool singleDistortion = true);
    void detectAll();
    void descriptAll();

    template <typename V>
    void accept(V &visitor)
    {
        ReconstructionParameters::accept(visitor);
        visitor.visit(scene,           MultiPhotostationScene(), "scene");
        visitor.visit(calibrationData, CalibrationJob(), "calibrationData");
    }

    struct ParallelUndistortionMapEstimator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const
        {
            for (int camId = r.begin(); camId < r.end(); ++camId)
            {
                auto& cam = (*photostation).cameras[camId];
                calibrationJob->prepareUndistortionTransformation(cam.distortion, cam.intrinsics.distortedSize[0], cam.intrinsics.distortedSize[1], (*transformation)[camId], cam.intrinsics.size[0], cam.intrinsics.size[1]);
            }
        }


        std::vector<corecvs::DisplacementBuffer> *transformations;
        CalibrationJob *calibrationJob;
        Photostation *photostation;
        
        ParallelUndistortionMapEstimator(decltype(transformations) transformations, CalibrationJob* calibrationJob, Photostation* photostation) : transformations(transformations), calibrationJob(calibrationJob), photostation(photostation)
        {
        }
    };

    struct ParallelUndistortionCalculator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const
        {
            for (int camId = r.begin(); camId < r.end(); ++camId)
            {
                auto& o = (*observations)[camId];
                auto& t = (*transformations)[camId];

                // Read file
                corecvs::RGB24Buffer src, dst;
                // Apply undistortion
                auto* res = o.doReverseDeformationBlTyped<corecvs::DisplacementBuffer>(&t, 
                // Write file

            }
        }

        std::vector<corecvs::DisplacementBuffer> *transformations;
        std::vector<CameraObservation> *observations;
    };
};

void ReconstructionJob::undistortAll(bool singleDistortion)
{
    int M = getPhotostationCount();
    std::vector<corecvs::DisplacementBuffer> transformations;
    for (int i = 0; i < M; ++i)
    {
        int N = photostations[i].cameras.size();
        if (i == 0 || !singleDistortion)
        {
            transformations.resize(N);
            corecvs::parallelable_for(0, N, ParallelUndistortionMapEstimator(&calibrationData, &transformations, photostations[i]);
        }
        corecvs::parallelable_for(0, N, ParallelUndistortionCalculator(cameraObservations[i], &transformations));
    }

    
}
#endif

// XXX: here we parse only position part; angle data is rejected
//      we also make prefix uppercase since it is partially lower-case
//      in Measure_15
std::unordered_map<std::string, LocationData>  parseGps(const std::string &filename)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios_base::in);
    if (!ifs)
    {
        std::cout << "GPS data file " << filename << " cannot be opened!" << std::endl;
        exit(0);
    }

    std::unordered_map<std::string, LocationData> locations;
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
        if(!std::regex_match(data, m, re) || m.size() < 5) // 4 tokens + final
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
        LocationData location;
        location.position[0] = e * 1e3;
        location.position[1] = n * 1e3;
        location.position[2] = h * 1e3;

        location.orientation = corecvs::Quaternion(0.25, 0.25, 0.25, 0.25);//corecvs::Quaternion(sin(M_PI / 4.0), 0.0, 0.0, cos(M_PI / 4.0));

        locations[key] = location;

        std::cout << "Key: " << key << " our stuff: " << location.position << " " << location.orientation << std::endl;
    } while(ifs);
    return locations;
}

int main(int argc, char **argv)
{
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

    auto map = parseGps(filenameGPS);

    CalibrationJob job;
    JSONGetter getter(filenameCalibration.c_str());
    getter.visit(job, "job");

    Photostation ps = job.photostation;

    Mesh3D mesh;
    mesh.switchColor(true);

    corecvs::Vector3dd sum(0.0, 0.0, 0.0), sumSq;
    for (auto& kp: map)
    {
        sum += kp.second.position;
        for (int i = 0; i < 3; ++i)
            sumSq[i] += kp.second.position[i]*kp.second.position[i];
    }
    sum /= (double)map.size();
    for (auto& kp: map)
    {
        std::cout << "KP: " << kp.second.position << " " << kp.second.orientation << std::endl;
        ps.location = kp.second;
        ps.location.position -= sum; // only for mesh output purposes
        std::cout << "Placing " << kp.first <<  ps.location.position << " " << ps.location.orientation << std::endl;
        ps.drawPly(mesh);
    }
    
    std::ofstream of;
    of.open(filenamePly, std::ios_base::out);
    mesh.dumpPLY(of);
    return 0;
}
