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

#include "calibrationHelpers.h"
#include "calibrationJob.h"
#include "jsonSetter.h"
#include "jsonGetter.h"

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
        CalibrationHelpers::drawPly(mesh, ps);
    }
    
    std::ofstream of;
    of.open(filenamePly, std::ios_base::out);
    mesh.dumpPLY(of);
    return 0;
}
